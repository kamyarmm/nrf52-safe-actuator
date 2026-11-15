//! The safety manager: one call per control cycle turns raw inputs into a state and a
//! bounded actuator command.

use crate::control::{clamp, PiController, RateLimiter};
use crate::fault::{FaultCode, FaultSet};
use crate::monitor::{in_range, tracking_error, Persistence};
use crate::state::{StateMachine, SystemState};

/// Tuning and limits. All positions are normalised to `0.0..=1.0`.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// Nominal control period in seconds.
    pub cycle_dt_s: f32,

    /// Sensor scaling: `position = (raw - sensor_offset) * sensor_gain`.
    pub sensor_offset: f32,
    /// See `sensor_offset`.
    pub sensor_gain: f32,
    /// Lower edge of the electrically valid raw band. Readings outside the band mean
    /// an open circuit or a short, not a position.
    pub raw_min: f32,
    /// Upper edge of the electrically valid raw band.
    pub raw_max: f32,
    /// Maximum age of a sensor sample before it is considered stale.
    pub max_sample_age_ms: u64,

    /// Reference envelope in normal operation.
    pub reference_min: f32,
    /// Upper reference bound in normal operation.
    pub reference_max: f32,
    /// Upper reference bound in degraded operation.
    pub degraded_reference_max: f32,
    /// Reference slew rate limit (position units per second), normal.
    pub max_rate: f32,
    /// Reference slew rate limit, degraded.
    pub degraded_max_rate: f32,
    /// Actuator command limit, normal.
    pub output_max: f32,
    /// Actuator command limit, degraded.
    pub degraded_output_max: f32,

    /// PI proportional gain.
    pub kp: f32,
    /// PI integral gain (1/s).
    pub ki: f32,

    /// Allowed |reference − position| before the tracking monitor trips.
    pub tracking_threshold: f32,

    /// Persistence `(set, clear)` cycles for each monitor.
    pub range_persistence: (u16, u16),
    /// See `range_persistence`.
    pub stale_persistence: (u16, u16),
    /// See `range_persistence`.
    pub tracking_persistence: (u16, u16),
    /// See `range_persistence`.
    pub overrun_persistence: (u16, u16),

    /// Fault-free time required to go from `Degraded` back to `Normal`.
    pub recovery_hold_ms: u64,
}

impl Config {
    /// Defaults matching the reference hardware (100 Hz loop, RC-filtered PWM plant
    /// behind a 2:1 divider to VDD, SAADC input pull-up enabled; see the README).
    ///
    /// The divider alone maps position 0..1 to 0.5..1.0 of full scale. The SAADC
    /// pull-up (~160 kΩ against the divider's 5 kΩ source impedance) pulls that
    /// slightly toward VDD: `raw = K·(0.5 + pos/2) + (1 − K)` with `K = 160/165`,
    /// i.e. `raw ≈ 0.5152 + 0.4848·pos`.
    pub const DEFAULT: Config = Config {
        cycle_dt_s: 0.01,
        sensor_offset: 0.5152,
        sensor_gain: 2.0625,
        raw_min: 0.45,
        raw_max: 0.95,
        max_sample_age_ms: 30,
        reference_min: 0.1,
        reference_max: 0.7,
        degraded_reference_max: 0.4,
        max_rate: 0.5,
        degraded_max_rate: 0.2,
        output_max: 0.8,
        degraded_output_max: 0.5,
        kp: 0.8,
        ki: 8.0,
        tracking_threshold: 0.1,
        range_persistence: (3, 10),
        stale_persistence: (2, 10),
        tracking_persistence: (30, 10),
        overrun_persistence: (3, 20),
        recovery_hold_ms: 2000,
    };
}

impl Default for Config {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Inputs for one control cycle.
#[derive(Clone, Copy, Debug)]
pub struct CycleInput {
    /// Monotonic time in milliseconds.
    pub now_ms: u64,
    /// Requested position.
    pub setpoint: f32,
    /// Latest raw sensor value as a fraction of ADC full scale, `None` if no sample
    /// has been received.
    pub raw: Option<f32>,
    /// Age of `raw`.
    pub sample_age_ms: u64,
    /// The previous cycle exceeded its execution budget.
    pub overrun: bool,
    /// Operator recovery request (edge).
    pub recovery_requested: bool,
}

/// Result of one control cycle.
#[derive(Clone, Copy, Debug)]
pub struct CycleOutput {
    /// State after this cycle.
    pub state: SystemState,
    /// Actuator command, `0.0` when not enabled.
    pub command: f32,
    /// Whether the actuator driver may be energised.
    pub output_enabled: bool,
    /// Scaled position, if the sample was usable.
    pub position: Option<f32>,
    /// Rate-limited reference actually used.
    pub reference: f32,
    /// All active faults (monitored and latched).
    pub active: FaultSet,
    /// Faults that became active this cycle; the caller logs these.
    pub new_faults: FaultSet,
}

/// Owns the monitors, controller and state machine.
#[derive(Clone, Debug)]
pub struct SafetyManager {
    cfg: Config,
    sm: StateMachine,
    pi: PiController,
    reference: RateLimiter,
    range: Persistence,
    stale: Persistence,
    tracking: Persistence,
    overrun: Persistence,
    latched: FaultSet,
    prev_active: FaultSet,
}

impl SafetyManager {
    /// Create a manager in `Init`.
    pub fn new(cfg: Config) -> Self {
        let p = |(s, c): (u16, u16)| Persistence::new(s, c);
        Self {
            sm: StateMachine::new(cfg.recovery_hold_ms),
            pi: PiController::new(cfg.kp, cfg.ki, 0.0, cfg.output_max),
            reference: RateLimiter::new(cfg.reference_min),
            range: p(cfg.range_persistence),
            stale: p(cfg.stale_persistence),
            tracking: p(cfg.tracking_persistence),
            overrun: p(cfg.overrun_persistence),
            latched: FaultSet::EMPTY,
            prev_active: FaultSet::EMPTY,
            cfg,
        }
    }

    /// Finish initialisation and enter `target` (from [`crate::state::boot_state`]).
    pub fn complete_init(&mut self, target: SystemState) {
        self.sm.complete_init(target);
    }

    /// Raise a fault detected outside the control loop (e.g. by the supervisor). It
    /// stays latched until an operator recovery request.
    pub fn raise(&mut self, code: FaultCode) {
        self.latched.insert(code);
    }

    /// Current state.
    pub fn state(&self) -> SystemState {
        self.sm.state()
    }

    /// Run one control cycle.
    pub fn step(&mut self, input: CycleInput) -> CycleOutput {
        let cfg = self.cfg;
        let was_enabled = self.sm.state().output_enabled();

        // ---- Monitors -------------------------------------------------------------
        let raw = input.raw.filter(|r| r.is_finite());
        let stale_bad = raw.is_none() || input.sample_age_ms > cfg.max_sample_age_ms;
        let range_bad = raw.is_some_and(|r| !in_range(r, cfg.raw_min, cfg.raw_max));
        let position = raw
            .filter(|_| !range_bad && !stale_bad)
            .map(|r| (r - cfg.sensor_offset) * cfg.sensor_gain);
        let track_bad = was_enabled
            && position
                .is_some_and(|p| tracking_error(self.reference.value(), p, cfg.tracking_threshold));

        let mut active = FaultSet::EMPTY;
        active.set(FaultCode::SensorOutOfRange, self.range.update(range_bad));
        active.set(FaultCode::SensorStale, self.stale.update(stale_bad));
        active.set(FaultCode::TrackingError, self.tracking.update(track_bad));
        active.set(
            FaultCode::DeadlineOverrun,
            self.overrun.update(input.overrun),
        );

        if input.recovery_requested && self.sm.state() == SystemState::SafeState {
            // Operator acknowledgement clears latched faults; if the cause persists
            // the reporter raises it again and recovery is refused.
            self.latched = FaultSet::EMPTY;
        }
        let active = active.union(self.latched);
        let new_faults = active.difference(self.prev_active);
        self.prev_active = active;

        // ---- State ----------------------------------------------------------------
        let state = self
            .sm
            .step(input.now_ms, active.worst(), input.recovery_requested);

        // ---- Output ---------------------------------------------------------------
        let (command, enabled) = if state.output_enabled() {
            if !was_enabled {
                // Bumpless start from wherever the actuator actually is.
                self.pi.reset();
                self.reference.reset(position.unwrap_or(cfg.reference_min));
            }
            let degraded = state == SystemState::Degraded;
            let (ref_max, rate, out_max) = if degraded {
                (
                    cfg.degraded_reference_max,
                    cfg.degraded_max_rate,
                    cfg.degraded_output_max,
                )
            } else {
                (cfg.reference_max, cfg.max_rate, cfg.output_max)
            };
            let target = clamp(input.setpoint, cfg.reference_min, ref_max);
            let r = self.reference.step(target, rate, cfg.cycle_dt_s);
            // Without a usable sample (before the stale monitor confirms) hold the
            // last output: zero error leaves only the integral term.
            let measured = position.unwrap_or(r);
            (self.pi.update(r, measured, cfg.cycle_dt_s, out_max), true)
        } else {
            self.pi.reset();
            if let Some(p) = position {
                self.reference.reset(p);
            }
            (0.0, false)
        };

        CycleOutput {
            state,
            command,
            output_enabled: enabled,
            position,
            reference: self.reference.value(),
            active,
            new_faults,
        }
    }
}
