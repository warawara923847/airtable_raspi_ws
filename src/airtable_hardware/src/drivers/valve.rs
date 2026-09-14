#[derive(Debug)]
pub struct ValveDriver {
    name: String,
    enabled: bool,
}

impl ValveDriver {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            enabled: false,
        }
    }

    pub fn set_enabled(&mut self, enabled: bool) -> Result<(), String> {
        if self.enabled == enabled {
            return Ok(());
        }

        self.enabled = enabled;

        println!(
            "[valve] {} -> {}",
            self.name,
            if enabled { "ON" } else { "OFF" }
        );

        Ok(())
    }
}
