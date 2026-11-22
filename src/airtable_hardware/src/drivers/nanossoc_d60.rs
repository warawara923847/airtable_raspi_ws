//! Solar MEMS nanoSSOC-D60 sun sensor over I2C.
//!
//! Ported from `airtable_esp/firmware/esp32_receiver/src/nanossoc_d60.cpp`: command 0x04,
//! repeated START, then a 15-byte frame
//! `[length, alpha f32, beta f32, intensity f32, error, checksum]` (little-endian), whose
//! checksum is the low byte of the sum of the first 14 bytes.
//!
//! Unlike the firmware, a failed read never keeps the previous angles or sun vector.
use std::f64::consts::TAU;
use std::io;
use std::time::Instant;

use crate::io::i2c::I2cBus;

pub const DEFAULT_ADDRESS: u8 = 0x64;

const CMD_ANGULAR_POSITION: u8 = 0x04;
const FRAME_SIZE: usize = 15;

pub const ERROR_NONE: u8 = 0x00;
pub const ERROR_TRANSMISSION_FAILED: u8 = 0x14;
pub const ERROR_FRAME_SIZE_MISMATCH: u8 = 0x15;
pub const ERROR_CHECKSUM_MISMATCH: u8 = 0x16;
pub const ERROR_INVALID_ANGLE: u8 = 0x17;

pub fn describe_error(code: u8) -> &'static str {
    match code {
        ERROR_NONE => "ok",
        0x10 => "not enough radiation",
        0x11 => "only albedo",
        0x12 => "both sun and albedo",
        0x13 => "out of field of view",
        ERROR_TRANSMISSION_FAILED => "I2C transfer failed (no ACK or bus error)",
        ERROR_FRAME_SIZE_MISMATCH => "short I2C transfer",
        ERROR_CHECKSUM_MISMATCH => "checksum mismatch",
        ERROR_INVALID_ANGLE => "angle is not finite or not below 90 deg",
        _ => "unknown sensor error code",
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SunObservation {
    pub alpha_deg: f32,
    pub beta_deg: f32,
    pub intensity_percent: f32,
    pub error_code: u8,
    /// Unit vector toward the sun in the sensor frame; only for a valid observation.
    pub sun_vector: Option<[f64; 3]>,
}

impl SunObservation {
    fn failed(error_code: u8) -> Self {
        Self {
            alpha_deg: f32::NAN,
            beta_deg: f32::NAN,
            intensity_percent: f32::NAN,
            error_code,
            sun_vector: None,
        }
    }
}

fn angle_to_vector(alpha_deg: f32, beta_deg: f32) -> Option<[f64; 3]> {
    if ![alpha_deg, beta_deg]
        .iter()
        .all(|angle| angle.is_finite() && angle.abs() < 90.0)
    {
        return None;
    }
    let x = f64::from(alpha_deg).to_radians().tan();
    let y = f64::from(beta_deg).to_radians().tan();
    let norm = (x * x + y * y + 1.0).sqrt();
    Some([x / norm, y / norm, 1.0 / norm])
}

fn checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0u8, |sum, byte| sum.wrapping_add(*byte))
}

fn parse_frame(frame: &[u8; FRAME_SIZE]) -> SunObservation {
    if checksum(&frame[..FRAME_SIZE - 1]) != frame[FRAME_SIZE - 1] {
        return SunObservation::failed(ERROR_CHECKSUM_MISMATCH);
    }
    let f32_at = |offset: usize| {
        f32::from_le_bytes(frame[offset..offset + 4].try_into().unwrap())
    };
    let (alpha_deg, beta_deg) = (f32_at(1), f32_at(5));
    let mut observation = SunObservation {
        alpha_deg,
        beta_deg,
        intensity_percent: f32_at(9),
        error_code: frame[13],
        sun_vector: None,
    };
    if observation.error_code == ERROR_NONE {
        observation.sun_vector = angle_to_vector(alpha_deg, beta_deg);
        if observation.sun_vector.is_none() {
            observation.error_code = ERROR_INVALID_ANGLE;
        }
    }
    observation
}

fn encode_frame(alpha_deg: f32, beta_deg: f32, intensity: f32, error: u8) -> [u8; FRAME_SIZE] {
    let mut frame = [0u8; FRAME_SIZE];
    frame[0] = (FRAME_SIZE - 1) as u8;
    frame[1..5].copy_from_slice(&alpha_deg.to_le_bytes());
    frame[5..9].copy_from_slice(&beta_deg.to_le_bytes());
    frame[9..13].copy_from_slice(&intensity.to_le_bytes());
    frame[13] = error;
    frame[14] = checksum(&frame[..14]);
    frame
}

pub struct NanoSsocD60 {
    bus: Box<dyn I2cBus>,
}

impl NanoSsocD60 {
    pub fn new(bus: Box<dyn I2cBus>) -> Self {
        Self { bus }
    }

    pub fn observe(&mut self) -> SunObservation {
        let mut frame = [0u8; FRAME_SIZE];
        match self.bus.write_read(&[CMD_ANGULAR_POSITION], &mut frame) {
            Ok(()) => parse_frame(&frame),
            Err(error) if error.kind() == io::ErrorKind::UnexpectedEof => {
                SunObservation::failed(ERROR_FRAME_SIZE_MISMATCH)
            }
            Err(_) => SunObservation::failed(ERROR_TRANSMISSION_FAILED),
        }
    }
}

/// `sun_sensor.backend: dummy`: answers the real protocol with a slowly moving sun, so the
/// parser, topic and display can be checked without the sensor.
pub struct DummySunBus {
    start: Instant,
}

impl DummySunBus {
    const PERIOD_S: f64 = 20.0;

    pub fn new() -> Self {
        Self {
            start: Instant::now(),
        }
    }
}

impl I2cBus for DummySunBus {
    fn write(&mut self, _data: &[u8]) -> io::Result<()> {
        Err(io::Error::from(io::ErrorKind::Unsupported))
    }

    fn read(&mut self, _buffer: &mut [u8]) -> io::Result<()> {
        Err(io::Error::from(io::ErrorKind::Unsupported))
    }

    fn write_read(&mut self, data: &[u8], buffer: &mut [u8]) -> io::Result<()> {
        if data != [CMD_ANGULAR_POSITION] || buffer.len() != FRAME_SIZE {
            return Err(io::Error::from(io::ErrorKind::InvalidInput));
        }
        let phase = TAU * self.start.elapsed().as_secs_f64() / Self::PERIOD_S;
        let frame = encode_frame(
            (40.0 * phase.sin()) as f32,
            (25.0 * phase.cos()) as f32,
            80.0,
            ERROR_NONE,
        );
        buffer.copy_from_slice(&frame);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    /// Replays one result and records what the host sent.
    struct ScriptedBus {
        reply: io::Result<[u8; FRAME_SIZE]>,
        sent: Arc<Mutex<Vec<Vec<u8>>>>,
    }

    impl I2cBus for ScriptedBus {
        fn write(&mut self, _data: &[u8]) -> io::Result<()> {
            panic!("the sensor needs a repeated START, not a separate write");
        }

        fn read(&mut self, _buffer: &mut [u8]) -> io::Result<()> {
            panic!("the sensor needs a repeated START, not a separate read");
        }

        fn write_read(&mut self, data: &[u8], buffer: &mut [u8]) -> io::Result<()> {
            self.sent.lock().unwrap().push(data.to_vec());
            match &self.reply {
                Ok(frame) => {
                    buffer.copy_from_slice(frame);
                    Ok(())
                }
                Err(error) => Err(io::Error::from(error.kind())),
            }
        }
    }

    fn observe(reply: io::Result<[u8; FRAME_SIZE]>) -> SunObservation {
        let sent = Arc::new(Mutex::new(Vec::new()));
        let bus = ScriptedBus {
            reply,
            sent: Arc::clone(&sent),
        };
        let observation = NanoSsocD60::new(Box::new(bus)).observe();
        assert_eq!(*sent.lock().unwrap(), [vec![CMD_ANGULAR_POSITION]]);
        observation
    }

    #[test]
    fn frame_layout_and_checksum_match_the_firmware() {
        let frame = encode_frame(1.0, -2.0, 50.0, ERROR_NONE);
        assert_eq!(&frame[1..5], &[0x00, 0x00, 0x80, 0x3f]);
        assert_eq!(&frame[5..9], &[0x00, 0x00, 0x00, 0xc0]);
        assert_eq!(&frame[9..13], &[0x00, 0x00, 0x48, 0x42]);
        let sum: u32 = frame[..14].iter().map(|byte| u32::from(*byte)).sum();
        assert_eq!(frame[14], (sum & 0xff) as u8);
    }

    #[test]
    fn valid_frame_gives_angles_and_a_unit_vector() {
        let observation = observe(Ok(encode_frame(30.0, -20.0, 87.5, ERROR_NONE)));
        assert_eq!(observation.alpha_deg, 30.0);
        assert_eq!(observation.beta_deg, -20.0);
        assert_eq!(observation.intensity_percent, 87.5);
        assert_eq!(observation.error_code, ERROR_NONE);
        let [x, y, z] = observation.sun_vector.unwrap();
        assert!(((x * x + y * y + z * z) - 1.0).abs() < 1e-12);
        assert!((x / z - 30f64.to_radians().tan()).abs() < 1e-12);
        assert!((y / z - (-20f64).to_radians().tan()).abs() < 1e-12);
    }

    #[test]
    fn zero_angles_point_along_the_boresight() {
        let observation = observe(Ok(encode_frame(0.0, 0.0, 100.0, ERROR_NONE)));
        assert_eq!(observation.sun_vector, Some([0.0, 0.0, 1.0]));
    }

    #[test]
    fn sensor_error_keeps_the_raw_angles_but_gives_no_direction() {
        let observation = observe(Ok(encode_frame(61.0, 2.0, 40.0, 0x13)));
        assert_eq!(observation.error_code, 0x13);
        assert_eq!(observation.alpha_deg, 61.0);
        assert_eq!(observation.sun_vector, None);
    }

    #[test]
    fn corrupted_frame_is_rejected() {
        let mut frame = encode_frame(10.0, 20.0, 30.0, ERROR_NONE);
        frame[3] ^= 0x01;
        let observation = observe(Ok(frame));
        assert_eq!(observation.error_code, ERROR_CHECKSUM_MISMATCH);
        assert!(observation.alpha_deg.is_nan());
        assert_eq!(observation.sun_vector, None);
    }

    #[test]
    fn impossible_angles_are_not_turned_into_a_direction() {
        for (alpha, beta) in [(f32::NAN, 0.0), (0.0, f32::INFINITY), (90.0, 0.0), (0.0, -91.0)] {
            let observation = observe(Ok(encode_frame(alpha, beta, 50.0, ERROR_NONE)));
            assert_eq!(observation.error_code, ERROR_INVALID_ANGLE);
            assert_eq!(observation.sun_vector, None);
        }
    }

    #[test]
    fn bus_failures_map_to_host_error_codes() {
        let absent = observe(Err(io::Error::from(io::ErrorKind::Other)));
        assert_eq!(absent.error_code, ERROR_TRANSMISSION_FAILED);
        assert!(absent.alpha_deg.is_nan());
        let short = observe(Err(io::Error::from(io::ErrorKind::UnexpectedEof)));
        assert_eq!(short.error_code, ERROR_FRAME_SIZE_MISMATCH);
    }

    #[test]
    fn dummy_bus_answers_through_the_real_parser() {
        let observation = NanoSsocD60::new(Box::new(DummySunBus::new())).observe();
        assert_eq!(observation.error_code, ERROR_NONE);
        assert!(observation.sun_vector.is_some());
    }
}
