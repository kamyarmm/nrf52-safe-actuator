//! Fault-tolerant actuator controller on the nRF52840-DK.
//!
//! See the repository README for the wiring, the safety concept and the
//! fault-injection walkthrough.
#![no_std]
#![no_main]

mod actuator;
mod persist;
mod shared;
mod tasks;

use defmt::{info, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::pwm::SimplePwm;
use embassy_nrf::saadc::{self, ChannelConfig, Gain, Reference, Saadc};
use embassy_nrf::wdt::{self, HaltConfig, Watchdog, WatchdogHandle};
use embassy_nrf::{bind_interrupts, peripherals};
use safety_core::state::boot_state;

use crate::actuator::Actuator;
use crate::tasks::ui::{Buttons, StatusLeds};

bind_interrupts!(struct Irqs {
    SAADC => saadc::InterruptHandler;
});

/// Hardware watchdog period. Must exceed the supervisor's worst-case detection time
/// (heartbeat timeout + one supervisor period) with margin.
const WATCHDOG_TICKS: u32 = 32_768 / 2; // 500 ms at 32.768 kHz

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    // ---- Why did we reset? ------------------------------------------------------------
    let cause = persist::take_reset_cause();
    let boot = persist::start_boot(cause);
    let target = boot_state(cause, boot.panic_in_previous_boot);
    info!(
        "boot #{} reset={} log_kept={} panic_prev={} -> start in {}",
        boot.boot, cause, boot.log_was_valid, boot.panic_in_previous_boot, target
    );
    persist::dump();

    // ---- Actuator: de-energised before anything else runs -----------------------------
    // P1.10 = actuator line (to RC plant), P0.13 = LED1 mirror.
    let pwm = SimplePwm::new_2ch(p.PWM0, p.P1_10, p.P0_13, &Actuator::config());
    shared::ACTUATOR.lock(|a| a.replace(Some(Actuator::new(pwm))));

    // ---- Watchdog ---------------------------------------------------------------------
    let mut wdt_cfg = wdt::Config::default();
    wdt_cfg.timeout_ticks = WATCHDOG_TICKS;
    // Pause while halted by a debugger so breakpoints do not reset the board.
    wdt_cfg.action_during_debug_halt = HaltConfig::Pause;
    let wdt_handle = match Watchdog::try_new::<_, 1>(p.WDT, wdt_cfg) {
        Ok((_wdt, [handle])) => handle,
        Err(_) => {
            // The WDT keeps running across a soft reset with its old configuration and
            // cannot be reconfigured until the next hard reset. Keep feeding it.
            warn!("watchdog already running with a different configuration");
            // SAFETY: the running watchdog was configured with one reload request
            // (handle 0) by a previous boot of this firmware.
            unsafe { WatchdogHandle::steal::<peripherals::WDT>(0) }
        }
    };

    // ---- Position sensor: AIN1 (P0.03), ratiometric to VDD ----------------------------
    let mut ch = ChannelConfig::single_ended(p.P0_03);
    ch.reference = Reference::VDD1_4;
    ch.gain = Gain::Gain1_4;
    let adc = Saadc::new(p.SAADC, Irqs, saadc::Config::default(), [ch]);

    // ---- Buttons and LEDs (nRF52840-DK) -----------------------------------------------
    let buttons = Buttons {
        recover: Input::new(p.P0_11, Pull::Up),
        freeze_sensor: Input::new(p.P0_12, Pull::Up),
        stall_control: Input::new(p.P0_24, Pull::Up),
        overrun: Input::new(p.P0_25, Pull::Up),
    };
    let leds = StatusLeds {
        state: Output::new(p.P0_14, Level::High, OutputDrive::Standard),
        injection: Output::new(p.P0_15, Level::High, OutputDrive::Standard),
        deenergized: Output::new(p.P0_16, Level::High, OutputDrive::Standard),
    };

    // ---- Tasks ------------------------------------------------------------------------
    spawner.spawn(tasks::supervisor::supervisor_task(wdt_handle).unwrap());
    spawner.spawn(tasks::sensor::sensor_task(adc).unwrap());
    spawner.spawn(tasks::control::control_task(target).unwrap());
    spawner.spawn(tasks::ui::button_task(buttons).unwrap());
    spawner.spawn(tasks::ui::led_task(leds).unwrap());
}
