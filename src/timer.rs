// In normal arduino code there is a millis() function that is used to time and debounce etc., however, such a 
// one does not seem to exist in rust / avr-hal so we use what we found written
// at: https://blog.rahix.de/005-avr-hal-millis/

use core::{cell, fmt::Display, ops::Sub};

use ufmt::{uDisplay, uwrite};

#[repr(transparent)]
#[derive(Clone, Copy, Default)]
pub struct MilliSeconds(u32);

impl Sub<Self> for MilliSeconds {
    type Output = u32;
    fn sub(self, rhs: Self) -> Self::Output {
        self.0 - rhs.0
    }
}

impl uDisplay for MilliSeconds {
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
        where
            W: ufmt::uWrite + ?Sized {
        uwrite!(f, "{}ms", self.0)
    }
}

const PRESCALER: u32 = 1024;
const TIMER_COUNTS: u32 = 125;
const MILLIS_INCREMENT: u32 = PRESCALER * TIMER_COUNTS / 16000;

static MILLIS_COUNTER: avr_device::interrupt::Mutex<cell::Cell<u32>> =
    avr_device::interrupt::Mutex::new(cell::Cell::new(0));

pub fn init_millis(tc0: arduino_hal::pac::TC0) {
    // Configure the timer for the above interval (in CTC mode)
    // and enable its interrupt.
    tc0.tccr0a.write(|w| w.wgm0().ctc());
    tc0.ocr0a.write(|w| w.bits(TIMER_COUNTS as u8));
    tc0.tccr0b.write(|w| match PRESCALER {
        8 => w.cs0().prescale_8(),
        64 => w.cs0().prescale_64(),
        256 => w.cs0().prescale_256(),
        1024 => w.cs0().prescale_1024(),
        _ => panic!(),
    });
    tc0.timsk0.write(|w| w.ocie0a().set_bit());

    // Reset the global millisecond counter
    avr_device::interrupt::free(|cs| {
        MILLIS_COUNTER.borrow(cs).set(0);
    });
}

pub fn millis() -> MilliSeconds {
    MilliSeconds(avr_device::interrupt::free(|cs| MILLIS_COUNTER.borrow(cs).get()))
}

#[inline(always)]
pub fn interrupt() {
    avr_device::interrupt::free(|cs| {
        let counter_cell = MILLIS_COUNTER.borrow(cs);
        let counter = counter_cell.get();
        counter_cell.set(counter + MILLIS_INCREMENT);
    })
}