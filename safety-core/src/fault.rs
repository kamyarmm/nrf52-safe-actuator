//! Fault identifiers, severities and a compact set type.

/// Every fault the system can detect or record.
///
/// Stored in the persistent log as a raw `u8`, so discriminants are stable and must
/// never be reused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum FaultCode {
    /// Position sensor reading is outside its electrically valid band
    /// (open circuit or short to a rail).
    SensorOutOfRange = 1,
    /// No fresh sensor sample within the allowed age.
    SensorStale = 2,
    /// Measured position does not follow the reference (actuator jammed,
    /// disconnected or driver failed).
    TrackingError = 3,
    /// The control cycle exceeded its execution budget.
    DeadlineOverrun = 4,
    /// A supervised task stopped reporting its heartbeat.
    HeartbeatLost = 5,
    /// Previous boot ended in a hardware watchdog reset.
    WatchdogReset = 6,
    /// Previous boot ended in a firmware panic.
    PanicReset = 7,
    /// Persistent fault log failed its integrity check and was reinitialised.
    LogCorrupted = 8,
}

impl FaultCode {
    /// All codes, in discriminant order.
    pub const ALL: [FaultCode; 8] = [
        FaultCode::SensorOutOfRange,
        FaultCode::SensorStale,
        FaultCode::TrackingError,
        FaultCode::DeadlineOverrun,
        FaultCode::HeartbeatLost,
        FaultCode::WatchdogReset,
        FaultCode::PanicReset,
        FaultCode::LogCorrupted,
    ];

    /// How the system must react while this fault is active.
    pub const fn severity(self) -> Severity {
        match self {
            FaultCode::SensorOutOfRange
            | FaultCode::SensorStale
            | FaultCode::TrackingError
            | FaultCode::HeartbeatLost => Severity::Critical,
            FaultCode::DeadlineOverrun => Severity::Degrading,
            FaultCode::WatchdogReset | FaultCode::PanicReset | FaultCode::LogCorrupted => {
                Severity::Info
            }
        }
    }

    /// Decode a raw log value. Returns `None` for unknown values, which can appear in a
    /// log written by a newer firmware version.
    pub const fn from_u8(raw: u8) -> Option<Self> {
        match raw {
            1 => Some(FaultCode::SensorOutOfRange),
            2 => Some(FaultCode::SensorStale),
            3 => Some(FaultCode::TrackingError),
            4 => Some(FaultCode::DeadlineOverrun),
            5 => Some(FaultCode::HeartbeatLost),
            6 => Some(FaultCode::WatchdogReset),
            7 => Some(FaultCode::PanicReset),
            8 => Some(FaultCode::LogCorrupted),
            _ => None,
        }
    }

    const fn bit(self) -> u16 {
        1 << (self as u8)
    }
}

/// Reaction class of a fault. Ordered: `Info < Degrading < Critical`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Severity {
    /// Recorded for diagnostics only; no change in behaviour.
    Info,
    /// Continue operating with reduced authority.
    Degrading,
    /// Remove actuator authority and latch the safe state.
    Critical,
}

/// A set of [`FaultCode`]s packed into a bitmask.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FaultSet(u16);

impl FaultSet {
    /// The empty set.
    pub const EMPTY: FaultSet = FaultSet(0);

    /// Add a fault.
    pub fn insert(&mut self, code: FaultCode) {
        self.0 |= code.bit();
    }

    /// Remove a fault.
    pub fn remove(&mut self, code: FaultCode) {
        self.0 &= !code.bit();
    }

    /// Set or clear a fault.
    pub fn set(&mut self, code: FaultCode, active: bool) {
        if active {
            self.insert(code)
        } else {
            self.remove(code)
        }
    }

    /// Membership test.
    pub const fn contains(self, code: FaultCode) -> bool {
        self.0 & code.bit() != 0
    }

    /// `true` if no fault is present.
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// Faults in `self` that are not in `other`.
    pub const fn difference(self, other: FaultSet) -> FaultSet {
        FaultSet(self.0 & !other.0)
    }

    /// Union of both sets.
    pub const fn union(self, other: FaultSet) -> FaultSet {
        FaultSet(self.0 | other.0)
    }

    /// Highest severity present, or `None` for an empty set.
    pub fn worst(self) -> Option<Severity> {
        self.iter().map(FaultCode::severity).max()
    }

    /// Iterate over the contained faults in discriminant order.
    pub fn iter(self) -> impl Iterator<Item = FaultCode> {
        FaultCode::ALL
            .into_iter()
            .filter(move |c| self.contains(*c))
    }

    /// Raw bitmask, e.g. for telemetry.
    pub const fn bits(self) -> u16 {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn raw_codes_round_trip() {
        for code in FaultCode::ALL {
            assert_eq!(FaultCode::from_u8(code as u8), Some(code));
        }
        assert_eq!(FaultCode::from_u8(0), None);
        assert_eq!(FaultCode::from_u8(200), None);
    }

    #[test]
    fn worst_severity_wins() {
        let mut set = FaultSet::EMPTY;
        assert_eq!(set.worst(), None);
        set.insert(FaultCode::WatchdogReset);
        assert_eq!(set.worst(), Some(Severity::Info));
        set.insert(FaultCode::DeadlineOverrun);
        assert_eq!(set.worst(), Some(Severity::Degrading));
        set.insert(FaultCode::SensorStale);
        assert_eq!(set.worst(), Some(Severity::Critical));
        set.remove(FaultCode::SensorStale);
        assert_eq!(set.worst(), Some(Severity::Degrading));
    }

    #[test]
    fn set_algebra() {
        let mut a = FaultSet::EMPTY;
        a.insert(FaultCode::SensorStale);
        a.insert(FaultCode::TrackingError);
        let mut b = FaultSet::EMPTY;
        b.insert(FaultCode::SensorStale);
        let d = a.difference(b);
        assert!(d.contains(FaultCode::TrackingError));
        assert!(!d.contains(FaultCode::SensorStale));
        assert_eq!(a.union(b), a);
        assert_eq!(a.iter().count(), 2);
    }
}
