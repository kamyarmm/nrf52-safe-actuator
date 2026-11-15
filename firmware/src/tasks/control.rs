//! 100 Hz control loop: runs the safety manager and drives the actuator.

use core::sync::atomic::Ordering;

use defmt::{info, warn};
#[cfg(feature = "fault-injection")]
use embassy_time::block_for;
use embassy_time::{Duration, Instant, Ticker};
use safety_core::{Config, CycleInput, FaultCode, SafetyManager, SystemState};

use crate::persist;
#[cfg(feature = "fault-injection")]
use crate::shared::inject;
use crate::shared::{
    self, TaskId, ACTUATOR, HEARTBEAT_FAULT, LATEST_SAMPLE, OUTPUT_INHIBIT, RECOVERY_REQUEST,
};

const PERIOD: Duration = Duration::from_millis(10);
/// Execution budget per cycle. Exceeding it is a `DeadlineOverrun`.
const BUDGET: Duration = Duration::from_millis(5);
const TELEMETRY_EVERY: u32 = 50; // cycles (0.5 s)
const INIT_TIMEOUT_CYCLES: u32 = 20; // 200 ms for the first sensor sample

#[embassy_executor::task]
pub async fn control_task(boot_state: SystemState) {
    let mut mgr = SafetyManager::new(Config::DEFAULT);
    let mut ticker = Ticker::every(PERIOD);

    // Init: power-up check that the sensor path is alive before enabling control.
    // Without it the first cycles would see no sample and latch a stale fault.
    let mut waited = 0;
    while LATEST_SAMPLE.lock(|s| s.get()).is_none() && waited < INIT_TIMEOUT_CYCLES {
        shared::heartbeat(TaskId::Control);
        ticker.next().await;
        waited += 1;
    }
    let target = if waited < INIT_TIMEOUT_CYCLES {
        boot_state
    } else {
        warn!("init: no sensor sample, entering safe state");
        SystemState::SafeState
    };
    mgr.complete_init(target);
    shared::publish_state(mgr.state());

    let mut overrun = false;
    let mut cycle: u32 = 0;

    loop {
        ticker.next().await;
        let start = Instant::now();

        #[cfg(feature = "fault-injection")]
        if inject::get(&inject::STALL_CONTROL) {
            warn!("[inject] control task stalled");
            // Never completes and never panics: invisible without supervision.
            core::future::pending::<()>().await;
        }
        shared::heartbeat(TaskId::Control);
        if HEARTBEAT_FAULT.swap(false, Ordering::Relaxed) {
            mgr.raise(FaultCode::HeartbeatLost);
        }

        let now = start.as_millis();
        let sample = LATEST_SAMPLE.lock(|s| s.get());
        let out = mgr.step(CycleInput {
            now_ms: now,
            setpoint: setpoint_profile(now),
            raw: sample.map(|s| s.raw),
            sample_age_ms: sample.map_or(u64::MAX, |s| {
                start.saturating_duration_since(s.at).as_millis()
            }),
            overrun,
            recovery_requested: RECOVERY_REQUEST.swap(false, Ordering::Relaxed),
        });

        let energise = out.output_enabled && !OUTPUT_INHIBIT.load(Ordering::Relaxed);
        ACTUATOR.lock(|a| {
            if let Some(a) = a.borrow_mut().as_mut() {
                if energise {
                    a.set(out.command)
                } else {
                    a.deenergize()
                }
            }
        });

        for f in out.new_faults.iter() {
            warn!("fault: {} -> {}", f, out.state);
            persist::record(f);
        }
        if out.state != shared::state() {
            info!("state: {} -> {}", shared::state(), out.state);
        }
        shared::publish_state(out.state);

        if cycle.is_multiple_of(TELEMETRY_EVERY) {
            info!(
                "{} ref={=f32} pos={} cmd={=f32} faults={=u16:#x}",
                out.state,
                out.reference,
                out.position,
                out.command,
                out.active.bits()
            );
        }
        cycle = cycle.wrapping_add(1);

        #[cfg(feature = "fault-injection")]
        if inject::get(&inject::OVERRUN) {
            block_for(Duration::from_millis(7));
        }
        overrun = start.elapsed() > BUDGET;
    }
}

/// Demonstration reference: 8 s triangle between 0.2 and 0.6.
fn setpoint_profile(now_ms: u64) -> f32 {
    let phase = (now_ms % 8000) as f32 / 8000.0;
    let tri = if phase < 0.5 {
        phase * 2.0
    } else {
        2.0 - phase * 2.0
    };
    0.2 + 0.4 * tri
}
