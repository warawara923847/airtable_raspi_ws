use crate::io::gpio::OutputLine;

/// Normally closed solenoid valve: output high opens it, low (and any shutdown) closes it.
pub struct ValveDriver {
    name: String,
    line: OutputLine,
    enabled: bool,
}

impl ValveDriver {
    /// The line must already be driven low, which is how `OutputLine` is created.
    pub fn new(name: impl Into<String>, line: OutputLine) -> Self {
        Self {
            name: name.into(),
            line,
            enabled: false,
        }
    }

    pub fn set_enabled(&mut self, enabled: bool) -> Result<(), String> {
        if self.enabled == enabled {
            return Ok(());
        }
        // On a write error `enabled` stays unchanged, so the caller's next attempt retries.
        self.line
            .set(enabled)
            .map_err(|error| format!("valve {}: {error}", self.name))?;
        self.enabled = enabled;
        if matches!(self.line, OutputLine::Dummy) {
            println!(
                "[valve] {} -> {}",
                self.name,
                if enabled { "ON" } else { "OFF" }
            );
        }
        Ok(())
    }
}

impl Drop for ValveDriver {
    fn drop(&mut self) {
        // Write unconditionally: after an earlier error the real state is unknown.
        match self.line.set(false) {
            Ok(()) => println!("[valve] {} closed at shutdown", self.name),
            Err(error) => eprintln!("[valve] {} FAILED to close at shutdown: {error}", self.name),
        }
    }
}
