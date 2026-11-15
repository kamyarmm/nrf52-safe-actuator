//! System state machine and boot policy.

use crate::fault::Severity;

/// Top-level operating state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SystemState {
    /// Power-up self checks; actuator de-energised.
    Init,
    /// Full authority.
    Normal,
    /// Reduced authority (output and reference envelope limited, slower rate).
    Degraded,
    /// Actuator de-energised. Latched: leaving requires all faults to be clear
    /// *and* an explicit operator recovery request.
    SafeState,
}

impl SystemState {
    /// Whether the actuator may be energised in this state.
    pub const fn output_enabled(self) -> bool {
        matches!(self, SystemState::Normal | SystemState::Degraded)
    }
}

/// Why the MCU last reset, as reported by the hardware.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ResetCause {
    /// Cold power-up.
    PowerOn,
    /// External reset pin.
    ResetPin,
    /// Hardware watchdog expired.
    Watchdog,
    /// CPU lockup (e.g. fault inside a fault handler).
    Lockup,
    /// Software-requested reset (also used by the panic handler).
    Software,
    /// Anything else (wake from system OFF, debug interface, ...).
    Other,
}

impl ResetCause {
    /// An unexpected reset means the previous boot did not shut down in a controlled
    /// way; the system must not resume actuation without operator acknowledgement.
    pub const fn is_unexpected(self) -> bool {
        matches!(self, ResetCause::Watchdog | ResetCause::Lockup)
    }
}

/// State after initialisation completes. `panic_recorded` comes from the persistent
/// fault log, because a panic resets through the same software-reset path as an
/// intentional reset.
pub const fn boot_state(cause: ResetCause, panic_recorded: bool) -> SystemState {
    if cause.is_unexpected() || panic_recorded {
        SystemState::SafeState
    } else {
        SystemState::Normal
    }
}

/// Deterministic state machine. All transitions happen in [`StateMachine::step`].
#[derive(Clone, Copy, Debug)]
pub struct StateMachine {
    state: SystemState,
    clear_since_ms: Option<u64>,
    recovery_hold_ms: u64,
}

impl StateMachine {
    /// Start in [`SystemState::Init`]. `recovery_hold_ms` is how long the system must
    /// be fault-free in `Degraded` before returning to `Normal`.
    pub const fn new(recovery_hold_ms: u64) -> Self {
        Self {
            state: SystemState::Init,
            clear_since_ms: None,
            recovery_hold_ms,
        }
    }

    /// Current state.
    pub const fn state(&self) -> SystemState {
        self.state
    }

    /// Leave `Init` for the state chosen by the boot policy.
    pub fn complete_init(&mut self, target: SystemState) {
        if self.state == SystemState::Init {
            self.state = match target {
                SystemState::Init => SystemState::SafeState,
                other => other,
            };
        }
    }

    /// Evaluate transitions for one cycle.
    ///
    /// * `worst` – highest severity among active faults (`None` if healthy).
    /// * `recovery_requested` – operator acknowledgement (edge, not level).
    pub fn step(
        &mut self,
        now_ms: u64,
        worst: Option<Severity>,
        recovery_requested: bool,
    ) -> SystemState {
        use SystemState::*;
        let critical = worst == Some(Severity::Critical);
        let degrading = worst == Some(Severity::Degrading);
        let healthy = !critical && !degrading;

        self.state = match self.state {
            Init => Init,
            _ if critical => SafeState,
            Normal if degrading => Degraded,
            Normal => Normal,
            Degraded if degrading => {
                self.clear_since_ms = None;
                Degraded
            }
            Degraded => {
                let since = *self.clear_since_ms.get_or_insert(now_ms);
                if now_ms.saturating_sub(since) >= self.recovery_hold_ms {
                    self.clear_since_ms = None;
                    Normal
                } else {
                    Degraded
                }
            }
            SafeState if healthy && recovery_requested => Normal,
            SafeState => SafeState,
        };
        if self.state != Degraded {
            self.clear_since_ms = None;
        }
        self.state
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use SystemState::*;

    fn running(target: SystemState) -> StateMachine {
        let mut sm = StateMachine::new(1000);
        sm.complete_init(target);
        sm
    }

    #[test]
    fn stays_in_init_until_completed() {
        let mut sm = StateMachine::new(1000);
        assert_eq!(sm.step(0, None, true), Init);
        sm.complete_init(Normal);
        assert_eq!(sm.step(1, None, false), Normal);
    }

    #[test]
    fn critical_latches_safe_state() {
        let mut sm = running(Normal);
        assert_eq!(sm.step(0, Some(Severity::Critical), false), SafeState);
        // Fault gone, but no operator request: stay latched.
        assert_eq!(sm.step(10_000, None, false), SafeState);
        // Request while a fault is still active is rejected.
        assert_eq!(sm.step(10_001, Some(Severity::Degrading), true), SafeState);
        assert_eq!(sm.step(10_002, None, true), Normal);
    }

    #[test]
    fn degraded_returns_to_normal_after_hold_time() {
        let mut sm = running(Normal);
        assert_eq!(sm.step(0, Some(Severity::Degrading), false), Degraded);
        assert_eq!(sm.step(100, None, false), Degraded);
        assert_eq!(sm.step(1099, None, false), Degraded);
        assert_eq!(sm.step(1100, None, false), Normal);
    }

    #[test]
    fn fault_reappearing_restarts_hold_timer() {
        let mut sm = running(Normal);
        sm.step(0, Some(Severity::Degrading), false);
        sm.step(100, None, false);
        sm.step(900, Some(Severity::Degrading), false);
        assert_eq!(sm.step(1200, None, false), Degraded);
        assert_eq!(sm.step(2199, None, false), Degraded);
        assert_eq!(sm.step(2200, None, false), Normal);
    }

    #[test]
    fn info_faults_do_not_change_state() {
        let mut sm = running(Normal);
        assert_eq!(sm.step(0, Some(Severity::Info), false), Normal);
    }

    #[test]
    fn boot_policy() {
        assert_eq!(boot_state(ResetCause::PowerOn, false), Normal);
        assert_eq!(boot_state(ResetCause::Software, false), Normal);
        assert_eq!(boot_state(ResetCause::Software, true), SafeState);
        assert_eq!(boot_state(ResetCause::Watchdog, false), SafeState);
        assert_eq!(boot_state(ResetCause::Lockup, false), SafeState);
    }
}
