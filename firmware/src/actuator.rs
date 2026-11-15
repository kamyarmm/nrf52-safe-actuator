//! PWM actuator driver.
//!
//! Channel 0 drives the actuator line (on the reference hardware an RC low-pass that
//! stands in for a motor driver); channel 1 mirrors the command on LED1 so the output
//! is visible on a bare DK.

use embassy_nrf::pwm::{DutyCycle, Prescaler, SimpleConfig, SimplePwm};

/// Counter top: 16 MHz / 1000 = 16 kHz PWM, well above the RC corner frequency.
const TOP: u16 = 1000;

pub struct Actuator {
    pwm: SimplePwm<'static>,
}

impl Actuator {
    pub fn config() -> SimpleConfig {
        let mut cfg = SimpleConfig::default();
        cfg.prescaler = Prescaler::Div1;
        cfg.max_duty = TOP;
        cfg
    }

    /// Takes a PWM already created with [`Actuator::config`]; starts de-energised.
    pub fn new(pwm: SimplePwm<'static>) -> Self {
        let mut a = Self { pwm };
        a.deenergize();
        a
    }

    /// Drive the actuator with a normalised command in `0.0..=1.0`.
    pub fn set(&mut self, command: f32) {
        let c = if command > 1.0 {
            1.0
        } else if command >= 0.0 {
            command
        } else {
            0.0 // also catches NaN
        };
        let v = (c * TOP as f32) as u16;
        self.pwm.set_all_duties([
            // Inverted polarity: pin high while counter < v, i.e. high for v/TOP.
            DutyCycle::inverted(v),
            // LED is active-low: normal polarity is low (LED on) while counter < v.
            DutyCycle::normal(v),
            DutyCycle::normal(0),
            DutyCycle::normal(0),
        ]);
    }

    /// Remove all drive: actuator line held low, LED off.
    pub fn deenergize(&mut self) {
        self.set(0.0);
    }
}
