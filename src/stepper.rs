use arduino_hal::port::{Pin, PinOps};


pub struct Stepper {
}

impl Stepper {
    pub fn new() -> Self {
        Self{}
    }

    // Abandon everything else ans simply stop moving as soon as possible, should
    // typically never be used as part of normal operations but only as a last resort
    // to avoid physical damage
    pub fn force_stop(&mut self) {
        todo!();
    }
}