//! Buttons (recovery + fault injection) and status LEDs.

use core::sync::atomic::Ordering;

use defmt::{info, warn};
use embassy_futures::select::{select4, Either4};
use embassy_nrf::gpio::{Input, Output};
use embassy_time::{Duration, Ticker, Timer};
use safety_core::SystemState;

use crate::persist;
use crate::shared::{self, inject, RECOVERY_REQUEST};

pub struct Buttons {
    pub recover: Input<'static>,
    pub freeze_sensor: Input<'static>,
    pub stall_control: Input<'static>,
    pub overrun: Input<'static>,
}

pub struct StatusLeds {
    /// Steady = Normal, slow blink = Degraded, fast blink = SafeState.
    pub state: Output<'static>,
    /// On while any fault injection is active.
    pub injection: Output<'static>,
    /// On while the actuator is de-energised.
    pub deenergized: Output<'static>,
}

#[embassy_executor::task]
pub async fn button_task(mut b: Buttons) {
    loop {
        let pressed = select4(
            b.recover.wait_for_falling_edge(),
            b.freeze_sensor.wait_for_falling_edge(),
            b.stall_control.wait_for_falling_edge(),
            b.overrun.wait_for_falling_edge(),
        )
        .await;

        match pressed {
            Either4::First(_) => {
                info!("operator: recovery requested");
                RECOVERY_REQUEST.store(true, Ordering::Relaxed);
                persist::dump();
            }
            Either4::Second(_) if cfg!(feature = "fault-injection") => {
                let on = inject::toggle(&inject::FREEZE_SENSOR);
                warn!("[inject] freeze sensor: {}", on);
            }
            Either4::Third(_) if cfg!(feature = "fault-injection") => {
                inject::STALL_CONTROL.store(true, Ordering::Relaxed);
                warn!("[inject] stall control task (expect watchdog reset)");
            }
            Either4::Fourth(_) if cfg!(feature = "fault-injection") => {
                let on = inject::toggle(&inject::OVERRUN);
                warn!("[inject] deadline overrun: {}", on);
            }
            _ => {}
        }
        Timer::after(Duration::from_millis(50)).await; // debounce
    }
}

#[embassy_executor::task]
pub async fn led_task(mut leds: StatusLeds) {
    let mut ticker = Ticker::every(Duration::from_millis(100));
    let mut tick: u32 = 0;
    loop {
        ticker.next().await;
        tick = tick.wrapping_add(1);
        let state = shared::state();
        // LEDs on the DK are active-low.
        let on = match state {
            SystemState::Init => false,
            SystemState::Normal => true,
            SystemState::Degraded => (tick / 5).is_multiple_of(2),
            SystemState::SafeState => tick.is_multiple_of(2),
        };
        leds.state.set_level((!on).into());
        leds.injection.set_level((!inject::any()).into());
        leds.deenergized.set_level(state.output_enabled().into());
    }
}
