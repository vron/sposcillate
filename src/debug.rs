use core::cell::RefCell;

use arduino_hal::{hal::port::{PD0, PD1}, pac::USART0, port::{mode::{Input, Output}, Pin}, Usart};
use avr_device::interrupt;
use ufmt::{uDisplay, uwriteln};

/// Simple module to do write some debugging infomration over a serial
/// interface when not compiled in release mode.

// Global lock to do the debug writing from any module
type Serial = Usart<USART0, Pin<Input, PD0>, Pin<Output, PD1>>;

static SERIAL: interrupt::Mutex<RefCell<Option<Serial>>> = interrupt::Mutex::new(RefCell::new(None));

pub fn init(s: Serial) {
    #[cfg(debug_assertions)]
      interrupt::free(|cs| {
        SERIAL.borrow(cs).replace(Some(s))
      });
}

pub fn debug<T: uDisplay>(ss: &str, s: T) {
    #[cfg(debug_assertions)]
    {
    // Does this really need to be interrupt free? Not sure - but for debugging only so who cares for now..
    interrupt::free(|cs| {
        let v = SERIAL.borrow(cs);
        if let Ok(mut v) = v.try_borrow_mut() {
            let v = v.as_mut();
            if let Some (v) = v {
                let _ = uwriteln!(v, "{}{}", ss, s);
            }
        }
        
    })
}
}