//! Fault monitors and the persistence filter that turns a noisy condition into a
//! confirmed fault.

/// Persistence (debounce) filter.
///
/// A fault is *confirmed* after the bad condition has been seen on `set_cycles`
/// consecutive evaluations, and *cleared* only after `clear_cycles` consecutive good
/// evaluations. This rejects single-sample glitches without hiding a real failure for
/// longer than a known, bounded detection time (`set_cycles × cycle period`).
#[derive(Clone, Copy, Debug)]
pub struct Persistence {
    set_cycles: u16,
    clear_cycles: u16,
    bad: u16,
    good: u16,
    active: bool,
}

impl Persistence {
    /// Create a filter. Both thresholds are clamped to at least 1.
    pub const fn new(set_cycles: u16, clear_cycles: u16) -> Self {
        Self {
            set_cycles: if set_cycles == 0 { 1 } else { set_cycles },
            clear_cycles: if clear_cycles == 0 { 1 } else { clear_cycles },
            bad: 0,
            good: 0,
            active: false,
        }
    }

    /// Feed one evaluation of the raw condition; returns the confirmed state.
    pub fn update(&mut self, condition_bad: bool) -> bool {
        if condition_bad {
            self.good = 0;
            self.bad = self.bad.saturating_add(1);
            if self.bad >= self.set_cycles {
                self.active = true;
            }
        } else {
            self.bad = 0;
            self.good = self.good.saturating_add(1);
            if self.good >= self.clear_cycles {
                self.active = false;
            }
        }
        self.active
    }

    /// Confirmed state without feeding a new evaluation.
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// Forget all history.
    pub fn reset(&mut self) {
        self.bad = 0;
        self.good = 0;
        self.active = false;
    }
}

/// `true` when `value` lies inside the closed band `[min, max]`. NaN is out of range.
pub fn in_range(value: f32, min: f32, max: f32) -> bool {
    value >= min && value <= max
}

/// `true` when the measurement deviates from the reference by more than `threshold`.
pub fn tracking_error(reference: f32, measured: f32, threshold: f32) -> bool {
    let err = (reference - measured).abs();
    // Written so that NaN (failed comparison) counts as an error.
    err.is_nan() || err > threshold
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn confirms_after_set_cycles_only() {
        let mut p = Persistence::new(3, 2);
        assert!(!p.update(true));
        assert!(!p.update(true));
        assert!(p.update(true));
    }

    #[test]
    fn glitch_resets_bad_counter() {
        let mut p = Persistence::new(3, 2);
        p.update(true);
        p.update(true);
        p.update(false); // single good sample breaks the run
        assert!(!p.update(true));
        assert!(!p.update(true));
        assert!(p.update(true));
    }

    #[test]
    fn clears_only_after_clear_cycles() {
        let mut p = Persistence::new(1, 3);
        assert!(p.update(true));
        assert!(p.update(false));
        assert!(p.update(false));
        assert!(!p.update(false));
    }

    #[test]
    fn zero_thresholds_are_clamped() {
        let mut p = Persistence::new(0, 0);
        assert!(p.update(true));
        assert!(!p.update(false));
    }

    #[test]
    fn range_and_tracking_treat_nan_as_fault() {
        assert!(in_range(0.5, 0.0, 1.0));
        assert!(!in_range(f32::NAN, 0.0, 1.0));
        assert!(!tracking_error(0.5, 0.55, 0.1));
        assert!(tracking_error(0.5, 0.7, 0.1));
        assert!(tracking_error(0.5, f32::NAN, 0.1));
    }
}
