#[derive(Debug)]
pub struct SimTable {
    name: String,
    body_id: usize,
}

impl SimTable {
    pub fn new(name: impl Into<String>, body_id: usize) -> Self {
        Self {
            name: name.into(),
            body_id,
        }
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn body_id(&self) -> usize {
        self.body_id
    }
}
