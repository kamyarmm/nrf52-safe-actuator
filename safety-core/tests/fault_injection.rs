//! Closed-loop failure-injection scenarios.
//!
//! A simulated plant reproduces the reference hardware: the PWM command is low-pass
//! filtered (first order, tau = 50 ms) and read back through a 2:1 divider to VDD, so
//! the healthy raw signal spans 0.5..0.9 of ADC full scale. Each scenario injects one
//! failure and checks the reaction *and* its detection-time bound.

use safety_core::state::boot_state;
use safety_core::{
    Config, CycleInput, CycleOutput, FaultCode, ResetCause, SafetyManager, SystemState,
};

const DT_MS: u64 = 10;
const TAU_S: f32 = 0.05;

#[derive(Clone, Copy, PartialEq)]
enum Injection {
    None,
    /// Sensor wire disconnected: divider pulls the input to VDD.
    SensorOpen,
    /// Sensor task stops delivering samples.
    SensorFrozen,
    /// Actuator mechanically stuck at its current position.
    ActuatorJammed,
    /// Control cycle exceeds its budget.
    Overrun,
    /// One-sample spike on the sensor line.
    SensorGlitch,
}

struct Sim {
    mgr: SafetyManager,
    now_ms: u64,
    plant: f32,
    last_sample_ms: u64,
    injection: Injection,
    recover_next: bool,
    log: Vec<FaultCode>,
}

impl Sim {
    fn new() -> Self {
        let mut mgr = SafetyManager::new(Config::DEFAULT);
        mgr.complete_init(boot_state(ResetCause::PowerOn, false));
        Self {
            mgr,
            now_ms: 0,
            plant: 0.0,
            last_sample_ms: 0,
            injection: Injection::None,
            recover_next: false,
            log: Vec::new(),
        }
    }

    fn setpoint(&self) -> f32 {
        // 8 s triangle between 0.2 and 0.6.
        let phase = (self.now_ms % 8000) as f32 / 8000.0;
        let tri = if phase < 0.5 {
            phase * 2.0
        } else {
            2.0 - phase * 2.0
        };
        0.2 + 0.4 * tri
    }

    fn step(&mut self) -> CycleOutput {
        self.now_ms += DT_MS;

        let healthy_raw = 0.5 + self.plant / 2.0;
        let raw = match self.injection {
            Injection::SensorOpen => Some(1.0),
            Injection::SensorGlitch => {
                self.injection = Injection::None;
                Some(0.0)
            }
            _ => Some(healthy_raw),
        };
        if self.injection != Injection::SensorFrozen {
            self.last_sample_ms = self.now_ms;
        }

        let out = self.mgr.step(CycleInput {
            now_ms: self.now_ms,
            setpoint: self.setpoint(),
            raw,
            sample_age_ms: self.now_ms - self.last_sample_ms,
            overrun: self.injection == Injection::Overrun,
            recovery_requested: core::mem::take(&mut self.recover_next),
        });
        self.log.extend(out.new_faults.iter());

        // Invariants that must hold in every cycle of every scenario.
        if !out.state.output_enabled() {
            assert_eq!(out.command, 0.0, "output energised in {:?}", out.state);
        }
        assert!(out.command <= Config::DEFAULT.output_max + 1e-6);
        if out.state == SystemState::Degraded {
            assert!(out.command <= Config::DEFAULT.degraded_output_max + 1e-6);
        }

        if self.injection != Injection::ActuatorJammed {
            self.plant += (out.command - self.plant) * (DT_MS as f32 / 1000.0) / TAU_S;
        }
        out
    }

    fn run_ms(&mut self, ms: u64) -> CycleOutput {
        let mut last = None;
        for _ in 0..ms / DT_MS {
            last = Some(self.step());
        }
        last.expect("ran at least one cycle")
    }

    /// Run until `pred` holds, returning the number of cycles taken.
    fn cycles_until(&mut self, max: u32, pred: impl Fn(&CycleOutput) -> bool) -> u32 {
        for n in 1..=max {
            if pred(&self.step()) {
                return n;
            }
        }
        panic!("condition not reached within {max} cycles");
    }
}

fn settled() -> Sim {
    let mut sim = Sim::new();
    sim.run_ms(2000);
    assert_eq!(sim.mgr.state(), SystemState::Normal);
    sim
}

#[test]
fn nominal_operation_tracks_without_faults() {
    let mut sim = Sim::new();
    let mut worst_err = 0.0f32;
    for _ in 0..2000 {
        let out = sim.step();
        assert_eq!(out.state, SystemState::Normal);
        assert!(out.active.is_empty(), "unexpected faults {:?}", out.active);
        if sim.now_ms > 1000 {
            worst_err = worst_err.max((out.reference - out.position.unwrap()).abs());
        }
    }
    assert!(worst_err < 0.03, "steady tracking error {worst_err}");
    assert!(sim.log.is_empty());
}

#[test]
fn open_sensor_reaches_safe_state_within_bound() {
    let mut sim = settled();
    sim.injection = Injection::SensorOpen;
    let n = sim.cycles_until(10, |o| o.state == SystemState::SafeState);
    assert!(n <= Config::DEFAULT.range_persistence.0 as u32);
    assert_eq!(sim.log, [FaultCode::SensorOutOfRange]);
}

#[test]
fn frozen_sensor_is_detected_as_stale() {
    let mut sim = settled();
    sim.injection = Injection::SensorFrozen;
    let n = sim.cycles_until(20, |o| o.state == SystemState::SafeState);
    // Age limit (30 ms = 3 cycles) plus stale persistence.
    let bound = (Config::DEFAULT.max_sample_age_ms / DT_MS) as u32
        + 1
        + Config::DEFAULT.stale_persistence.0 as u32;
    assert!(n <= bound, "took {n} cycles, bound {bound}");
    assert!(sim.log.contains(&FaultCode::SensorStale));
}

#[test]
fn jammed_actuator_trips_tracking_monitor() {
    let mut sim = settled();
    sim.injection = Injection::ActuatorJammed;
    // The reference keeps moving at 0.1/s on the triangle; once the error exceeds the
    // threshold, detection must follow within the persistence time.
    sim.cycles_until(300, |o| o.state == SystemState::SafeState);
    assert_eq!(sim.log, [FaultCode::TrackingError]);
}

#[test]
fn single_sample_glitch_is_rejected() {
    let mut sim = settled();
    sim.injection = Injection::SensorGlitch;
    let out = sim.run_ms(500);
    assert_eq!(out.state, SystemState::Normal);
    assert!(sim.log.is_empty());
}

#[test]
fn overruns_degrade_then_recover_automatically() {
    let mut sim = settled();
    sim.injection = Injection::Overrun;
    let n = sim.cycles_until(10, |o| o.state == SystemState::Degraded);
    assert_eq!(n, Config::DEFAULT.overrun_persistence.0 as u32);

    // Stay degraded with limited authority while overruns continue.
    let out = sim.run_ms(1000);
    assert_eq!(out.state, SystemState::Degraded);
    assert!(out.reference <= Config::DEFAULT.degraded_reference_max + 1e-6);

    sim.injection = Injection::None;
    let clear_ms = Config::DEFAULT.overrun_persistence.1 as u64 * DT_MS;
    let out = sim.run_ms(clear_ms + Config::DEFAULT.recovery_hold_ms + DT_MS);
    assert_eq!(out.state, SystemState::Normal);
}

#[test]
fn safe_state_is_latched_until_operator_recovery() {
    let mut sim = settled();
    sim.injection = Injection::SensorOpen;
    sim.run_ms(100);
    sim.injection = Injection::None;

    // Fault cleared, but no acknowledgement: stays safe.
    let out = sim.run_ms(3000);
    assert_eq!(out.state, SystemState::SafeState);
    assert!(out.active.is_empty());

    sim.recover_next = true;
    let before = sim.plant;
    let out = sim.step();
    assert_eq!(out.state, SystemState::Normal);
    // Bumpless restart: the reference starts at the measured position.
    assert!((out.reference - before).abs() < 0.02);
    let out = sim.run_ms(2000);
    assert_eq!(out.state, SystemState::Normal);
}

#[test]
fn recovery_is_refused_while_fault_persists() {
    let mut sim = settled();
    sim.injection = Injection::SensorOpen;
    sim.run_ms(100);
    sim.recover_next = true;
    let out = sim.step();
    assert_eq!(out.state, SystemState::SafeState);
}

#[test]
fn external_heartbeat_fault_cuts_output_in_same_cycle() {
    let mut sim = settled();
    sim.mgr.raise(FaultCode::HeartbeatLost);
    let out = sim.step();
    assert_eq!(out.state, SystemState::SafeState);
    assert_eq!(out.command, 0.0);
    assert_eq!(sim.log, [FaultCode::HeartbeatLost]);

    // Acknowledged and the supervisor does not re-raise: recovery succeeds.
    sim.recover_next = true;
    assert_eq!(sim.step().state, SystemState::Normal);
}

#[test]
fn boots_into_safe_state_after_watchdog_reset() {
    let mut mgr = SafetyManager::new(Config::DEFAULT);
    mgr.complete_init(boot_state(ResetCause::Watchdog, false));
    let out = mgr.step(CycleInput {
        now_ms: 10,
        setpoint: 0.5,
        raw: Some(0.5),
        sample_age_ms: 0,
        overrun: false,
        recovery_requested: false,
    });
    assert_eq!(out.state, SystemState::SafeState);
    assert!(!out.output_enabled);
}
