use embedded_hal::digital::{InputPin, OutputPin};
// Simple wrapper module to controll our LED screen to display messages to the user
use tm1637_embedded_hal::{tokens::Blocking, Brightness, TM1637Builder, TM1637};
use arduino_hal::{Delay};

pub struct Screen< CLK, DIO> {
    tm: TM1637<4 ,Blocking, CLK, DIO, Delay>
}

impl<CLK, DIO, E>  Screen<CLK, DIO>  where 
    CLK: OutputPin<Error = E>, 
    DIO: OutputPin<Error = E> {
    pub fn new(clk: CLK, dio: DIO) -> Self {
        let delay = Delay::new();
        let mut tm = TM1637Builder::new(clk, dio, delay)
        .brightness(Brightness::L1)
        .delay_us(100)
        .build_blocking::<4>();
        tm.init().ok();
        Self {tm}
    }

    pub fn show(&mut self, s: &str) {
        let e = self.tm.options().str(s).display();
        #[cfg(debug_assertions)]
        if let Err(e) = e {
            let _ = e; //TODO:Write debug to serial interface
        }
    }
}