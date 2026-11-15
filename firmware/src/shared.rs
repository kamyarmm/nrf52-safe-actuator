//! State shared between tasks.
//!
//! Everything runs on one thread-mode executor, so contention is only with
//! interrupts; small values use atomics and the rest uses a critical-section mutex.

use core::cell::{Cell, RefCell};
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::Instant;
use safety_core::SystemState;

use crate::actuator::Actuator;

/// Latest position-sensor sample.
#[derive(Clone, Copy)]
pub struct Sample {
    /// Fraction of ADC full scale (ratiometric to VDD).
    pub raw: f32,
    /// When it was taken.
    pub at: Instant,
}

pub static LATEST_SAMPLE: Mutex<CriticalSectionRawMutex, Cell<Option<Sample>>> =
    Mutex::new(Cell::new(None));

/// The actuator driver lives in a mutex so the supervisor can de-energise it even when
/// the control task is no longer running.
pub static ACTUATOR: Mutex<CriticalSectionRawMutex, RefCell<Option<Actuator>>> =
    Mutex::new(RefCell::new(None));

/// Set by the supervisor; once set, nothing may energise the actuator until reset.
pub static OUTPUT_INHIBIT: AtomicBool = AtomicBool::new(false);

/// Operator recovery request (button 1), consumed by the control task.
pub static RECOVERY_REQUEST: AtomicBool = AtomicBool::new(false);

/// Current [`SystemState`] for the status LEDs.
static STATE: AtomicU8 = AtomicU8::new(0);

pub fn publish_state(s: SystemState) {
    STATE.store(s as u8, Ordering::Relaxed);
}

pub fn state() -> SystemState {
    match STATE.load(Ordering::Relaxed) {
        1 => SystemState::Normal,
        2 => SystemState::Degraded,
        3 => SystemState::SafeState,
        _ => SystemState::Init,
    }
}

/// Supervised tasks.
#[derive(Clone, Copy)]
pub enum TaskId {
    Sensor = 0,
    Control = 1,
}

pub const SUPERVISED_TASKS: usize = 2;
pub const TASK_NAMES: [&str; SUPERVISED_TASKS] = ["sensor", "control"];

/// Last heartbeat of each supervised task, in ms since boot.
static HEARTBEATS: [AtomicU32; SUPERVISED_TASKS] = [AtomicU32::new(0), AtomicU32::new(0)];

pub fn heartbeat(task: TaskId) {
    HEARTBEATS[task as usize].store(now_ms() as u32, Ordering::Relaxed);
}

pub fn last_heartbeat_ms(task: usize) -> u64 {
    HEARTBEATS[task].load(Ordering::Relaxed) as u64
}

pub fn now_ms() -> u64 {
    Instant::now().as_millis()
}

/// Fault-injection switches driven by the buttons. Compiled in only with the
/// `fault-injection` feature (on by default for this demonstrator).
pub mod inject {
    use core::sync::atomic::{AtomicBool, Ordering};

    /// Sensor task keeps running but stops publishing samples (stuck ADC/DMA).
    pub static FREEZE_SENSOR: AtomicBool = AtomicBool::new(false);
    /// Control task awaits forever without crashing (lost wake-up, deadlock).
    pub static STALL_CONTROL: AtomicBool = AtomicBool::new(false);
    /// Control task burns CPU past its execution budget each cycle.
    pub static OVERRUN: AtomicBool = AtomicBool::new(false);

    pub fn get(flag: &AtomicBool) -> bool {
        cfg!(feature = "fault-injection") && flag.load(Ordering::Relaxed)
    }

    pub fn toggle(flag: &AtomicBool) -> bool {
        !flag.fetch_xor(true, Ordering::Relaxed)
    }

    pub fn any() -> bool {
        get(&FREEZE_SENSOR) || get(&STALL_CONTROL) || get(&OVERRUN)
    }
}
