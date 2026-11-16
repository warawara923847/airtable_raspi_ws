//! ASPINA ARW-3m reaction wheel protocol over I2C (ICD v1.4).
//!
//! Ported from the C sample `airtable_raspi/src/drivers/arw3m.c`; packet layouts and
//! offsets follow that file. Not yet exercised against a real wheel.
use std::time::{Duration, Instant};

use crate::io::i2c::I2cBus;

pub const DEFAULT_ADDRESS: u8 = 0x60;

const CMD_PACKET_SIZE: usize = 22;
const RESP_PACKET_SIZE: usize = 24;
const DATA_REQ_PACKET_SIZE: usize = 7;
const ADD_DATA_PACKET_SIZE: usize = 71;
const ADD_DATA_BODY_SIZE: u8 = 64;
const ADD_DATA_HEADER_SIZE: usize = 6;

const CMD_TELEMETRY: u8 = 0x01;
const CMD_SET_TORQUE: u8 = 0x02;
const CMD_START_STOP: u8 = 0x04;
const CMD_ERROR_CLEAR: u8 = 0x05;
const CMD_SOFT_RESET: u8 = 0x07;
const CMD_DATA_REQUEST: u8 = 0x42;
const CMD_ADDITIONAL_DATA: u8 = 0x43;

// The wheel NACKs while it prepares a reply; each phase is polled up to this long.
const PHASE_TIMEOUT: Duration = Duration::from_millis(10);
const POLL_INTERVAL: Duration = Duration::from_micros(200);
const TELEMETRY_ATTEMPTS: u32 = 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Arw3mError {
    /// No ACK or a bus error for the whole phase timeout.
    I2c,
    Checksum,
    /// The wheel answered with a negative execution result or an unexpected header.
    Exec,
}

impl std::fmt::Display for Arw3mError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::I2c => "I2C transfer failed or timed out",
            Self::Checksum => "checksum mismatch",
            Self::Exec => "wheel reported an execution error or unexpected reply",
        })
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Telemetry {
    pub wheel_time_ms: u32,
    pub error_status: u32,
    pub rotation_speed_rad_s: f32,
    pub total_reset_count: u16,
    pub motor_current_a: f32,
    pub motor_voltage_v: f32,
    pub bearing_temp_c: f32,
    pub motor_coil_temp_c: f32,
    pub mcu_temp_c: f32,
    pub total_rotations: u64,
    pub torque_nm: f32,
    /// 0: stop / coasting, 1: running.
    pub control_status: u32,
}

/// Recommended handling of an error status (ICD v1.4 section 6.1), most severe last.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum RecoveryAction {
    None,
    /// Keep operating; log only.
    A,
    /// 0x05 error clear, then 0x04 start.
    B,
    /// 0x07 soft reset, then 0x04 start.
    C,
}

// (bit, description, action) for the 32-bit error status (ICD v1.4 sections 5.1 and 6).
const ERROR_BITS: [(u32, &str, RecoveryAction); 16] = [
    (0, "forced motor stop by MCU", RecoveryAction::A),
    (4, "rotation sensor error", RecoveryAction::B),
    (5, "driver hardware error", RecoveryAction::C),
    (6, "hardware over-current protection", RecoveryAction::B),
    (8, "coil over-temperature", RecoveryAction::A),
    (9, "bearing over-temperature", RecoveryAction::A),
    (10, "MCU over-temperature", RecoveryAction::A),
    (11, "circuit over-temperature", RecoveryAction::A),
    (12, "current deviation error", RecoveryAction::A),
    (13, "speed deviation error", RecoveryAction::A),
    (14, "over-speed", RecoveryAction::A),
    (16, "SRAM parity error", RecoveryAction::A),
    (17, "motor lock", RecoveryAction::C),
    (18, "motor voltage error", RecoveryAction::C),
    (24, "over-current measured by MCU", RecoveryAction::C),
    (
        25,
        "rotation sensor short-period toggling",
        RecoveryAction::C,
    ),
];

pub fn recovery_action(error_status: u32) -> RecoveryAction {
    ERROR_BITS
        .iter()
        .filter(|(bit, _, _)| error_status & (1 << bit) != 0)
        .map(|(_, _, action)| *action)
        .max()
        .unwrap_or(RecoveryAction::None)
}

pub fn describe_errors(error_status: u32) -> Vec<&'static str> {
    ERROR_BITS
        .iter()
        .filter(|(bit, _, _)| error_status & (1 << bit) != 0)
        .map(|(_, text, _)| *text)
        .collect()
}

/// Checksum byte that makes the 8-bit sum of the whole packet zero.
pub fn checksum(packet: &[u8]) -> u8 {
    0u8.wrapping_sub(packet.iter().fold(0u8, |sum, byte| sum.wrapping_add(*byte)))
}

pub fn checksum_ok(packet: &[u8]) -> bool {
    !packet.is_empty() && packet.iter().fold(0u8, |sum, byte| sum.wrapping_add(*byte)) == 0
}

fn command_packet(sequence: u16, command: u8, data: [u32; 4]) -> [u8; CMD_PACKET_SIZE] {
    let mut packet = [0u8; CMD_PACKET_SIZE];
    packet[0..2].copy_from_slice(&sequence.to_le_bytes());
    packet[2] = command;
    // packet[3]: additional data size, always 0 for these commands; packet[20] is unused.
    for (index, word) in data.iter().enumerate() {
        packet[4 + 4 * index..8 + 4 * index].copy_from_slice(&word.to_le_bytes());
    }
    packet[CMD_PACKET_SIZE - 1] = checksum(&packet[..CMD_PACKET_SIZE - 1]);
    packet
}

fn u32_at(packet: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(packet[offset..offset + 4].try_into().unwrap())
}

fn f32_at(packet: &[u8], offset: usize) -> f32 {
    f32::from_bits(u32_at(packet, offset))
}

pub struct Arw3m {
    bus: Box<dyn I2cBus>,
    sequence: u16,
}

impl Arw3m {
    pub fn new(bus: Box<dyn I2cBus>) -> Self {
        Self { bus, sequence: 0 }
    }

    fn poll(
        &mut self,
        mut transfer: impl FnMut(&mut dyn I2cBus) -> std::io::Result<()>,
    ) -> Result<(), Arw3mError> {
        let start = Instant::now();
        loop {
            if transfer(self.bus.as_mut()).is_ok() {
                return Ok(());
            }
            if start.elapsed() >= PHASE_TIMEOUT {
                return Err(Arw3mError::I2c);
            }
            std::thread::sleep(POLL_INTERVAL);
        }
    }

    fn next_sequence(&mut self) -> u16 {
        self.sequence = self.sequence.wrapping_add(1);
        self.sequence
    }

    /// Send a 22-byte command and receive and check the 24-byte 0x40 response.
    fn command(
        &mut self,
        sequence: u16,
        command: u8,
        data: [u32; 4],
    ) -> Result<[u8; RESP_PACKET_SIZE], Arw3mError> {
        let packet = command_packet(sequence, command, data);
        self.poll(|bus| bus.write(&packet))?;
        let mut response = [0u8; RESP_PACKET_SIZE];
        self.poll(|bus| bus.read(&mut response))?;
        if !checksum_ok(&response) {
            return Err(Arw3mError::Checksum);
        }
        // Execution result is an int8: negative means the wheel rejected the command.
        if (response[3] as i8) < 0 {
            return Err(Arw3mError::Exec);
        }
        Ok(response)
    }

    pub fn control_start(&mut self) -> Result<(), Arw3mError> {
        let sequence = self.next_sequence();
        self.command(sequence, CMD_START_STOP, [1, 0, 0, 0])
            .map(|_| ())
    }

    /// Stop driving; the wheel coasts.
    pub fn control_stop(&mut self) -> Result<(), Arw3mError> {
        let sequence = self.next_sequence();
        self.command(sequence, CMD_START_STOP, [0, 0, 0, 0])
            .map(|_| ())
    }

    /// Set the torque target [Nm] with a speed limit [rad/s], applied immediately.
    pub fn set_torque(&mut self, torque_nm: f32, speed_limit_rad_s: f32) -> Result<(), Arw3mError> {
        let sequence = self.next_sequence();
        let data = [torque_nm.to_bits(), speed_limit_rad_s.to_bits(), 1, 0];
        self.command(sequence, CMD_SET_TORQUE, data).map(|_| ())
    }

    #[allow(dead_code)] // Recovery action B; not issued automatically yet.
    pub fn clear_error(&mut self) -> Result<(), Arw3mError> {
        let sequence = self.next_sequence();
        self.command(sequence, CMD_ERROR_CLEAR, [0; 4]).map(|_| ())
    }

    /// The wheel restarts at once and sends no response.
    #[allow(dead_code)] // Recovery action C; not issued automatically yet.
    pub fn soft_reset(&mut self) -> Result<(), Arw3mError> {
        let sequence = self.next_sequence();
        let packet = command_packet(sequence, CMD_SOFT_RESET, [1, 0, 0, 0]);
        self.poll(|bus| bus.write(&packet))
    }

    fn telemetry_once(&mut self) -> Result<Telemetry, Arw3mError> {
        let sequence = self.next_sequence();
        let response = self.command(sequence, CMD_TELEMETRY, [0; 4])?;
        if response[4] != ADD_DATA_BODY_SIZE {
            return Err(Arw3mError::Exec);
        }

        // 0x42 DATA_REQUEST with the same sequence ID, then the 71-byte 0x43 packet.
        let mut request = [0u8; DATA_REQ_PACKET_SIZE];
        request[0..2].copy_from_slice(&sequence.to_le_bytes());
        request[2] = CMD_DATA_REQUEST;
        request[3] = 2;
        request[4] = CMD_TELEMETRY;
        request[5] = ADD_DATA_BODY_SIZE;
        request[6] = checksum(&request[..6]);
        self.poll(|bus| bus.write(&request))?;
        let mut data = [0u8; ADD_DATA_PACKET_SIZE];
        self.poll(|bus| bus.read(&mut data))?;
        if !checksum_ok(&data) {
            return Err(Arw3mError::Checksum);
        }
        if data[2] != CMD_ADDITIONAL_DATA
            || data[3] != 66
            || data[4] != CMD_TELEMETRY
            || data[5] != ADD_DATA_BODY_SIZE
        {
            return Err(Arw3mError::Exec);
        }

        let body = ADD_DATA_HEADER_SIZE;
        Ok(Telemetry {
            wheel_time_ms: u32_at(&response, 5),
            error_status: u32_at(&response, 9),
            rotation_speed_rad_s: f32_at(&response, 13),
            total_reset_count: u16::from_le_bytes([response[17], response[18]]),
            motor_current_a: f32_at(&data, body),
            motor_voltage_v: f32_at(&data, body + 4),
            // body + 8: average acceleration, not used.
            bearing_temp_c: f32_at(&data, body + 12),
            motor_coil_temp_c: f32_at(&data, body + 16),
            mcu_temp_c: f32_at(&data, body + 20),
            total_rotations: u64::from(u32_at(&data, body + 24))
                | (u64::from(u32_at(&data, body + 28)) << 32),
            torque_nm: f32_at(&data, body + 32),
            control_status: u32_at(&data, body + 36),
        })
    }

    /// 0x01 telemetry request; the whole four-transfer sequence is retried on error.
    pub fn telemetry(&mut self) -> Result<Telemetry, Arw3mError> {
        let mut last = Arw3mError::I2c;
        for _ in 0..TELEMETRY_ATTEMPTS {
            match self.telemetry_once() {
                Ok(telemetry) => return Ok(telemetry),
                Err(error) => last = error,
            }
            std::thread::sleep(POLL_INTERVAL * 2);
        }
        Err(last)
    }
}

/// Simulated wheel for tests: answers the protocol and integrates the torque command.
#[cfg(test)]
pub mod sim {
    use super::*;
    use std::io;
    use std::sync::{Arc, Mutex};

    #[derive(Debug, Default)]
    pub struct WheelModel {
        pub running: bool,
        pub torque_nm: f32,
        pub speed_limit_rad_s: f32,
        pub speed_rad_s: f32,
        pub error_status: u32,
        pub writes: Vec<Vec<u8>>,
        /// Number of upcoming transfers to NACK.
        pub nack: u32,
        /// Corrupt the checksum of the next reply.
        pub corrupt_next: bool,
        /// Never acknowledge, like an absent or unpowered wheel.
        pub absent: bool,
        pub resets: u32,
    }

    pub struct SimBus {
        pub model: Arc<Mutex<WheelModel>>,
        reply: Vec<u8>,
    }

    impl SimBus {
        pub fn new() -> (Self, Arc<Mutex<WheelModel>>) {
            let model = Arc::new(Mutex::new(WheelModel::default()));
            (
                Self {
                    model: Arc::clone(&model),
                    reply: Vec::new(),
                },
                model,
            )
        }
    }

    fn nacked(model: &mut WheelModel) -> bool {
        if model.absent {
            return true;
        }
        if model.nack > 0 {
            model.nack -= 1;
            return true;
        }
        false
    }

    impl I2cBus for SimBus {
        fn write(&mut self, data: &[u8]) -> io::Result<()> {
            let mut model = self.model.lock().unwrap();
            if nacked(&mut model) {
                return Err(io::Error::other("NACK"));
            }
            assert!(checksum_ok(data), "host sent a bad checksum");
            model.writes.push(data.to_vec());
            let sequence = [data[0], data[1]];
            if data[2] == CMD_DATA_REQUEST {
                assert_eq!(data.len(), DATA_REQ_PACKET_SIZE);
                let mut reply = vec![0u8; ADD_DATA_PACKET_SIZE];
                reply[0..2].copy_from_slice(&sequence);
                reply[2..6].copy_from_slice(&[CMD_ADDITIONAL_DATA, 66, data[4], data[5]]);
                reply[6..10].copy_from_slice(&0.25f32.to_le_bytes());
                reply[10..14].copy_from_slice(&12.0f32.to_le_bytes());
                reply[18..22].copy_from_slice(&31.5f32.to_le_bytes());
                reply[22..26].copy_from_slice(&32.5f32.to_le_bytes());
                reply[26..30].copy_from_slice(&33.5f32.to_le_bytes());
                reply[30..38].copy_from_slice(&0x1_0000_0002u64.to_le_bytes());
                let torque = if model.running { model.torque_nm } else { 0.0 };
                reply[38..42].copy_from_slice(&torque.to_le_bytes());
                reply[42..46].copy_from_slice(&u32::from(model.running).to_le_bytes());
                reply[70] = checksum(&reply[..70]);
                self.reply = reply;
                return Ok(());
            }
            assert_eq!(data.len(), CMD_PACKET_SIZE);
            let word = |index: usize| u32_at(data, 4 + 4 * index);
            let mut reply = vec![0u8; RESP_PACKET_SIZE];
            reply[0..2].copy_from_slice(&sequence);
            reply[2] = data[2];
            match data[2] {
                CMD_TELEMETRY => {
                    reply[4] = ADD_DATA_BODY_SIZE;
                    reply[5..9].copy_from_slice(&1234u32.to_le_bytes());
                    reply[9..13].copy_from_slice(&model.error_status.to_le_bytes());
                    reply[13..17].copy_from_slice(&model.speed_rad_s.to_le_bytes());
                    reply[17..19].copy_from_slice(&7u16.to_le_bytes());
                }
                CMD_SET_TORQUE => {
                    model.torque_nm = f32::from_bits(word(0));
                    model.speed_limit_rad_s = f32::from_bits(word(1));
                    assert_eq!(word(2), 1, "torque must be applied immediately");
                }
                CMD_START_STOP => model.running = word(0) == 1,
                CMD_ERROR_CLEAR => model.error_status = 0,
                CMD_SOFT_RESET => {
                    *model = WheelModel {
                        resets: model.resets + 1,
                        writes: std::mem::take(&mut model.writes),
                        ..WheelModel::default()
                    };
                    self.reply.clear();
                    return Ok(());
                }
                _ => reply[3] = 0xff, // execution result -1
            }
            reply[23] = checksum(&reply[..23]);
            self.reply = reply;
            Ok(())
        }

        fn read(&mut self, buffer: &mut [u8]) -> io::Result<()> {
            let mut model = self.model.lock().unwrap();
            if nacked(&mut model) || self.reply.len() != buffer.len() {
                return Err(io::Error::other("NACK"));
            }
            buffer.copy_from_slice(&self.reply);
            self.reply.clear();
            if model.corrupt_next {
                buffer[5] ^= 0x01;
                model.corrupt_next = false;
            }
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::sim::SimBus;
    use super::*;

    #[test]
    fn command_packet_matches_the_icd_layout() {
        let packet = command_packet(
            0x1234,
            CMD_SET_TORQUE,
            [0.0005f32.to_bits(), 628.3f32.to_bits(), 1, 0],
        );
        assert_eq!(packet.len(), 22);
        assert_eq!(&packet[..4], &[0x34, 0x12, 0x02, 0x00]);
        assert_eq!(&packet[4..8], &0.0005f32.to_le_bytes());
        assert_eq!(&packet[8..12], &628.3f32.to_le_bytes());
        assert_eq!(&packet[12..16], &[1, 0, 0, 0]);
        assert_eq!(&packet[16..21], &[0; 5]);
        assert!(checksum_ok(&packet));
        // Same arithmetic as the C sample: checksum = 0 - sum.
        assert_eq!(checksum(&[0x01, 0xff, 0x10]), 0xf0);
        assert!(!checksum_ok(&[]));
    }

    #[test]
    fn start_torque_telemetry_and_stop_round_trip() {
        let (bus, model) = SimBus::new();
        let mut wheel = Arw3m::new(Box::new(bus));
        wheel.control_start().unwrap();
        wheel.set_torque(-0.0005, 628.3).unwrap();
        model.lock().unwrap().speed_rad_s = -42.5;
        model.lock().unwrap().error_status = 1 << 9;
        let telemetry = wheel.telemetry().unwrap();
        assert_eq!(
            telemetry,
            Telemetry {
                wheel_time_ms: 1234,
                error_status: 1 << 9,
                rotation_speed_rad_s: -42.5,
                total_reset_count: 7,
                motor_current_a: 0.25,
                motor_voltage_v: 12.0,
                bearing_temp_c: 31.5,
                motor_coil_temp_c: 32.5,
                mcu_temp_c: 33.5,
                total_rotations: 0x1_0000_0002,
                torque_nm: -0.0005,
                control_status: 1,
            }
        );
        wheel.control_stop().unwrap();
        let model = model.lock().unwrap();
        assert!(!model.running);
        assert_eq!(model.speed_limit_rad_s, 628.3);
        // start, torque, telemetry, data request, stop; sequence IDs 1, 2, 3, 3, 4.
        let sequences: Vec<u8> = model.writes.iter().map(|packet| packet[0]).collect();
        assert_eq!(sequences, [1, 2, 3, 3, 4]);
        assert_eq!(model.writes[3], [3, 0, 0x42, 2, 0x01, 64, 0x78]);
    }

    #[test]
    fn transmitted_bytes_match_the_c_sample() {
        // Recorded from airtable_raspi/src/drivers/arw3m.c (commit of 2026-11) driven through
        // a recording I2C stub with the same call sequence.
        const C_SAMPLE: [&str; 7] = [
            "010004000100000000000000000000000000000000fa", // control start
            "020002006f1203ba33131d4401000000000000000016", // torque -0.0005 Nm, 628.3 rad/s
            "030001000000000000000000000000000000000000fc", // telemetry request
            "03004202014078",                               // data request
            "040004000000000000000000000000000000000000f8", // control stop
            "050005000000000000000000000000000000000000f6", // error clear
            "060007000100000000000000000000000000000000f2", // soft reset
        ];
        let (bus, model) = SimBus::new();
        let mut wheel = Arw3m::new(Box::new(bus));
        wheel.control_start().unwrap();
        wheel.set_torque(-0.0005, 628.3).unwrap();
        wheel.telemetry().unwrap();
        wheel.control_stop().unwrap();
        wheel.clear_error().unwrap();
        wheel.soft_reset().unwrap();
        let sent: Vec<String> = model
            .lock()
            .unwrap()
            .writes
            .iter()
            .map(|packet| packet.iter().map(|byte| format!("{byte:02x}")).collect())
            .collect();
        assert_eq!(sent, C_SAMPLE);
    }

    #[test]
    fn busy_wheel_is_polled_and_bad_checksum_is_retried_or_reported() {
        let (bus, model) = SimBus::new();
        let mut wheel = Arw3m::new(Box::new(bus));
        model.lock().unwrap().nack = 5;
        wheel.control_start().unwrap();
        // One corrupted reply: telemetry retries the whole sequence and succeeds.
        model.lock().unwrap().corrupt_next = true;
        assert!(wheel.telemetry().is_ok());
        // Commands are not retried; the caller sees the checksum error.
        model.lock().unwrap().corrupt_next = true;
        assert_eq!(wheel.set_torque(0.0, 1.0), Err(Arw3mError::Checksum));
    }

    #[test]
    fn absent_wheel_times_out_instead_of_blocking() {
        let (bus, model) = SimBus::new();
        model.lock().unwrap().absent = true;
        let mut wheel = Arw3m::new(Box::new(bus));
        let start = Instant::now();
        assert_eq!(wheel.control_start(), Err(Arw3mError::I2c));
        assert_eq!(wheel.telemetry(), Err(Arw3mError::I2c));
        assert!(start.elapsed() < Duration::from_millis(200));
    }

    #[test]
    fn clear_and_reset_commands_reach_the_wheel() {
        let (bus, model) = SimBus::new();
        let mut wheel = Arw3m::new(Box::new(bus));
        model.lock().unwrap().error_status = 1 << 4;
        wheel.clear_error().unwrap();
        assert_eq!(model.lock().unwrap().error_status, 0);
        wheel.control_start().unwrap();
        wheel.soft_reset().unwrap();
        assert_eq!(model.lock().unwrap().resets, 1);
        assert!(!model.lock().unwrap().running);
    }

    #[test]
    fn error_bits_map_to_the_most_severe_recovery_action() {
        assert_eq!(recovery_action(0), RecoveryAction::None);
        assert_eq!(recovery_action(1 << 9), RecoveryAction::A);
        assert_eq!(recovery_action(1 << 0 | 1 << 16), RecoveryAction::A);
        assert_eq!(recovery_action(1 << 9 | 1 << 4), RecoveryAction::B);
        assert_eq!(recovery_action(1 << 6), RecoveryAction::B);
        for bit in [5, 17, 18, 24, 25] {
            assert_eq!(recovery_action(1 << bit | 1 << 4), RecoveryAction::C);
        }
        // Bits the ICD does not define are ignored.
        assert_eq!(recovery_action(1 << 1 | 1 << 31), RecoveryAction::None);
        assert_eq!(
            describe_errors(1 << 17 | 1 << 8),
            ["coil over-temperature", "motor lock"]
        );
    }
}
