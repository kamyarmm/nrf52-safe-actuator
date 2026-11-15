//! Task supervision and the hardware watchdog.
//!
//! The watchdog is fed only while every supervised task is reporting heartbeats. On a
//! missing heartbeat the supervisor de-energises the actuator itself, without relying
//! on the control task, records the fault, and stops feeding the watchdog so the MCU
//! resets and boots into the latched safe state. It shares the PWM driver and the
//! executor with the control task, so it is not independent of a blocked executor:
//! in that case this task cannot run either, the PWM keeps its last duty, and the
//! hardware watchdog (up to 500 ms) is the backstop.

use core::sync::atomic::Ordering;

use defmt::error;
use embassy_nrf::wdt::WatchdogHandle;
use embassy_time::{Duration, Ticker};
use safety_core::{FaultCode, HeartbeatMonitor};

use crate::persist;
use crate::shared::{
    self, ACTUATOR, HEARTBEAT_FAULT, OUTPUT_INHIBIT, SUPERVISED_TASKS, TASK_NAMES,
};

const PERIOD: Duration = Duration::from_millis(20);
/// Per-task heartbeat timeout: [sensor, control]. Several periods of margin each.
const TIMEOUTS_MS: [u64; SUPERVISED_TASKS] = [50, 50];

#[embassy_executor::task]
pub async fn supervisor_task(mut wdt: WatchdogHandle) {
    let mut hb = HeartbeatMonitor::new(TIMEOUTS_MS, shared::now_ms());
    let mut ticker = Ticker::every(PERIOD);
    let mut tripped = false;

    loop {
        ticker.next().await;
        let now = shared::now_ms();
        for task in 0..SUPERVISED_TASKS {
            hb.kick(
                task,
                now.saturating_sub(shared::heartbeat_age_ms(task, now)),
            );
        }

        match hb.check(now) {
            Ok(()) if !tripped => wdt.pet(),
            Ok(()) => {}
            Err(task) if !tripped => {
                tripped = true;
                error!(
                    "heartbeat lost: {} task; de-energising and withholding watchdog",
                    TASK_NAMES[task]
                );
                OUTPUT_INHIBIT.store(true, Ordering::SeqCst);
                HEARTBEAT_FAULT.store(true, Ordering::SeqCst);
                ACTUATOR.lock(|a| {
                    if let Some(a) = a.borrow_mut().as_mut() {
                        a.deenergize();
                    }
                });
                persist::record(FaultCode::HeartbeatLost);
            }
            Err(_) => {}
        }
    }
}
