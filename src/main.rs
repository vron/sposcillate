// env RAVEDUDE_PORT=/dev/tty.SLAB_USBtoUART cargo run
#![no_std]
#![no_main]
#![feature(likely_unlikely)]
#![feature(abi_avr_interrupt)]

mod buttons;
mod debug;
mod screen;
mod stepper;
mod timer;

use buttons::{ButtonEvent, Buttons};
use screen::Screen;
use stepper::Stepper;

#[avr_device::interrupt(atmega328p)]
fn TIMER0_COMPA() {
    timer::interrupt();
}

use panic_halt as _;

const STEP_PER_MM: i32 = 1024; // Key configuration, how many steps do we signal for 1 millimeter motion of the spindle
const LARGE_STEP: i32 = STEP_PER_MM; // How big a large step is, typically 1mm
const SMALL_STEP: i32 = STEP_PER_MM / 10; // how many steps do a small click correspond to
const MAX_SPEED: i32 = STEP_PER_MM * 4; // Max velocity of the spindle ever
const BASE_SPEED: i32 = STEP_PER_MM * 2; // Base oscilating speed
const MIN_DIST: i32 = STEP_PER_MM * 5; // Minimum distance between lower and upper turning point
const MAX_UPPER: i32 = i32::MAX; // MAximum vertical pos to ensure no overflow
const OSC_MOVE: i32 = 5 * STEP_PER_MM; // How much the oscillation endpoints should move on one click

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    // Create a serial interface to allow us to debug stuff to the computer in debug mode
    debug::init(arduino_hal::default_serial!(dp, pins, 57600));

    // The state variables representing tagets etc. Much of this should be stored to and
    // reloaded from eeperom
    let mut state = State::Manual;
    let mut current_pos = 0 as i32; // The current position represented in steps
    let mut target_pos = current_pos; // The target position represented in steps
    let mut target_vel = 0 as i32;
    let mut oscilating_vel = BASE_SPEED;
    let mut saved_pos = 0 as i32;
    let mut end_points = (0 as i32, 1024 as i32); // (lower, upper) end-point for oscillaion in steps
    let mut last_event_shift = false;

    // Initiate the timer, including enabling interrupts
    timer::init_millis(dp.TC0);
    unsafe { avr_device::interrupt::enable() };

    // Create the button structure that we will use for events
    let mut btns: Buttons<_, _, _, _, 5, 10, 500> = Buttons::new(
        pins.d9.into_pull_up_input(),
        pins.d10.into_pull_up_input(),
        pins.d11.into_pull_up_input(),
        pins.d12.into_pull_up_input(),
    );
    const BTN_UP: u8 = 0;
    const BTN_DOWN: u8 = 1;
    const BTN_START_STOP: u8 = 2;
    const BTN_OPTION: u8 = 3;

    // Initiate the stepper engine such that we can move stuff as we want
    let mut stepper = Stepper::new();

    // Initiate the screen so we can display stuff to the user
    let mut screen = Screen::new(pins.a5.into_output(), pins.a4.into_output());

    // Prepare the input to read the extrem position sensors
    let extreme = pins.d8.into_pull_up_input();

    debug::debug("", "will start loop");

    'outer: loop {
        // First check the extrem limits sensors and kick into locked state if so
        // TODO: Consider also introducing an interrupt to force the state as soon as the sensor triggers
        if state == State::Locked || extreme.is_low() {
            // force everything into safe modes and then loop forever
            stepper.force_stop();
            screen.show("LOC ");
            loop {
                arduino_hal::delay_ms(100);
            }
        }

        let now = timer::millis();
        // The main state-machine is changed from user interaction, i.e.
        // events as well as events from the sensors - i.e. this part is were
        // all the user interaction happens.
        // Since we have a key-coord (click shift then up or down) we need to
        // keep a state across. TODO: We really want to make a generic module
        // to handle keycoords etc. instead of this spiderweb - but have not
        // figured out a good design for that yet
        let last_shift = last_event_shift;
        last_event_shift = false;
        target_vel = 0;
        for e in btns.run(now) {
            match e {
                ButtonEvent::Click { btn, mods } => {
                    match *btn {
                        BTN_UP => {
                            match state {
                                State::Manual => {
                                    if last_shift {
                                        if current_pos >= saved_pos {
                                            // Move it all the way up, the soft limit will stop it
                                            target_pos = MAX_UPPER;
                                            debug::debug("move to saved", "");
                                        } else {
                                            // Move it to the saved position
                                            target_pos = saved_pos;
                                            debug::debug("move to top", "");
                                        }
                                    } else {
                                        if mods.pressed(BTN_DOWN) {
                                            // Save the target position
                                            saved_pos = current_pos;
                                            debug::debug("position saved", saved_pos);
                                        } else {
                                            // Move it up a little bit
                                            target_pos += if mods.pressed(BTN_OPTION) {
                                                SMALL_STEP
                                            } else {
                                                LARGE_STEP
                                            };
                                            debug::debug("move up", "");
                                        }
                                    }
                                }
                                State::Oscillating => {
                                    if mods.pressed(BTN_START_STOP) {
                                        // Increase the oscilating velocity with 1mm/s
                                        oscilating_vel =
                                            MAX_SPEED.min(oscilating_vel + STEP_PER_MM);
                                        debug::debug("increase o. vel ", oscilating_vel);
                                    } else if mods.pressed(BTN_OPTION) {
                                        // Move the lower turning point up
                                        end_points.0 = MAX_UPPER
                                            .min(end_points.0 + OSC_MOVE)
                                            .min(end_points.1 - MIN_DIST);
                                        debug::debug("move up lower", end_points.0);
                                    } else if mods.empty() {
                                        // Move the upper turning point up - within reason
                                        end_points.1 = (MAX_UPPER).min(end_points.1 + OSC_MOVE);
                                        debug::debug("move up upper", end_points.1);
                                    }
                                }
                                State::Locked => continue 'outer,
                            }
                        }
                        BTN_DOWN => {
                            match state {
                                State::Manual => {
                                    if last_shift {
                                        // Move it all the way down, the limit will stop it
                                        target_pos = i32::MIN;
                                        debug::debug("move to bottom", "");
                                    } else {
                                        if mods.pressed(BTN_UP) {
                                            // Save the target position
                                            saved_pos = current_pos;
                                            debug::debug("position saved", saved_pos);
                                        } else {
                                            // Move it down a little bit
                                            target_pos -= if mods.pressed(BTN_OPTION) {
                                                SMALL_STEP
                                            } else {
                                                LARGE_STEP
                                            };
                                            debug::debug("move down", "");
                                        }
                                    }
                                }
                                State::Oscillating => {
                                    if mods.pressed(BTN_START_STOP) {
                                        // Decrease the oscilating velocity
                                        oscilating_vel =
                                            STEP_PER_MM.max(oscilating_vel - STEP_PER_MM);
                                        debug::debug("decrease o. vel ", oscilating_vel);
                                    } else if mods.pressed(BTN_OPTION) {
                                        // Move the lower turning point down
                                        end_points.0 = 0.max(end_points.0 - OSC_MOVE);
                                        debug::debug("move down lower", end_points.0);
                                    } else if mods.empty() {
                                        // Move the upper turning point down
                                        end_points.1 = 0
                                            .max(end_points.1 - OSC_MOVE)
                                            .max(end_points.0 + MIN_DIST);
                                        debug::debug("move down upper", end_points.1);
                                    }
                                }
                                State::Locked => continue 'outer,
                            }
                        }
                        BTN_START_STOP => {
                            state = match state {
                                State::Manual => {
                                    debug::debug("start osc.", "");
                                    State::Oscillating
                                }
                                State::Oscillating => {
                                    debug::debug("start osc.", "");
                                    State::Manual
                                }
                                State::Locked => continue 'outer,
                            }
                        }
                        BTN_OPTION => {
                            // If the user clicks shift and only shift it might be the start of a coord
                            if mods.empty() && state == State::Manual {
                                last_event_shift = true;
                                debug::debug("starting coord", "");
                            }
                        }
                        b => debug::debug("unknown button click: ", b),
                    }
                }
                ButtonEvent::Press { btn, millis, mods } => match *btn {
                    BTN_UP => match state {
                        State::Manual => {
                            target_vel = MAX_SPEED;
                        }
                        State::Oscillating => {}
                        State::Locked => continue 'outer,
                    },
                    BTN_DOWN => match state {
                        State::Manual => {
                            target_vel = -MAX_SPEED;
                        }
                        State::Oscillating => {}
                        State::Locked => continue 'outer,
                    },
                    BTN_START_STOP => {}
                    BTN_OPTION => {}
                    b => debug::debug("unknown button press: ", b),
                },
            }
        }

        // But regardless of what the user tells us we will listen to the soft
        // limit sensors and react to them - i.e. possibly overriding what the user
        // tells us to do
        // Whenever we change here we must keep the invariants
    }
}

// The program / spindle is considered to possibly be in a couple
// of different states as represented below
#[derive(PartialEq, Eq)]
enum State {
    // Locked state means that one of the extreme limits sensors has
    // tripped and that we therefore will not do anything futher before
    // a restart (and a manual motion of the spindle so that the sensore
    // no longer truips)
    Locked,
    // In manual mode there is not automatic motion of the spindle, only
    // motion of the spindle as a consequence of pressing the move buttons
    Manual,
    // In Oscillating mode the spindle will move periodically up and down
    // between the two set-points (or end-points if hit before)
    Oscillating,
}

enum SpindleTarget {
    Position(i32), // Target position in steps
    Velocity(i32), // Target velocity up or down
}

/*

impl SposcillateApp {

    pub fn main(&mut self) {
        // Main application loop beeing run, not that since the stepper eninge is beeing controlled
        // in this loop we cannot accept long delays anywhere (or we will get uneven moion)
        loop {
            self.handle_buttons();
            self.update_display();

        }
    }

    // We check the button state by polling the digital state of the corresponding input pins
    fn handle_buttons(&mut self) {


    if let Some(e) = self.buttons.run() {
        use ButtonEvent::*;
        use State::*;
        match e {
            Click(b_start_stop, m) => match self.state {
                Started => self.state = Stopped,
                Stopped => self.state = Started,
            }
            Click(b_up, m) => self.target_pos += 1,
            Click(b_down, m) => self.target_pos -= 1,
            TODO - REALLY WANT 1 PER BUTTON TO ENSURE ALL HANDLED!
        }

    }
}
}


 */

/*

void loop()
{
  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  if (currentMillis - startMillis >= period)  //test whether the period has elapsed
  {
    digitalWrite(ledPin, !digitalRead(ledPin));  //if so, change the state of the LED.  Uses a neat trick to change the state
    startMillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.
  }
}

*/
