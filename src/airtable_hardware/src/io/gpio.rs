#[derive(Debug, Clone, Copy)]
pub enum Direction {
    Positive,
    Negative,
}

#[derive(Debug)]
pub struct DummyGpio {
    axis_name: String,
}

impl DummyGpio {
    pub fn new(axis_name: impl Into<String>) -> Self {
        Self {
            axis_name: axis_name.into(),
        }
    }

    pub fn set_direction(&mut self, direction: Direction) {
        match direction {
            Direction::Positive => {
                println!("[io] {} DIR = positive", self.axis_name);
            }
            Direction::Negative => {
                println!("[io] {} DIR = negative", self.axis_name);
            }
        }
    }

    pub fn pulse_steps(&mut self, steps: u64) {
        println!("[io] {} STEP x{}", self.axis_name, steps);
    }
}
