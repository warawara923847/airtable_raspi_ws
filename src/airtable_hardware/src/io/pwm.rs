#[derive(Debug)]
pub struct DummyPwm {
    channel: u8,
    last_pulse_width_us: u16,
}

impl DummyPwm {
    pub fn new(channel: u8) -> Self {
        Self {
            channel,
            last_pulse_width_us: 0,
        }
    }

    pub fn set_pulse_width_us(&mut self, pulse_width_us: u16) -> Result<(), String> {
        self.last_pulse_width_us = pulse_width_us;

        println!(
            "[io] PWM channel {}: {} us",
            self.channel, self.last_pulse_width_us
        );

        Ok(())
    }
}
