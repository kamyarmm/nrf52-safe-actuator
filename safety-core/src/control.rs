//! Position controller building blocks.

/// PI controller with output clamping and conditional-integration anti-windup.
#[derive(Clone, Copy, Debug)]
pub struct PiController {
    kp: f32,
    ki: f32,
    out_min: f32,
    out_max: f32,
    integral: f32,
}

impl PiController {
    /// Create a controller whose output is clamped to `[out_min, out_max]`.
    pub const fn new(kp: f32, ki: f32, out_min: f32, out_max: f32) -> Self {
        Self {
            kp,
            ki,
            out_min,
            out_max,
            integral: 0.0,
        }
    }

    /// Run one step. `dt` is in seconds. `limit` further restricts the upper output
    /// bound (used for reduced authority in degraded mode).
    pub fn update(&mut self, reference: f32, measured: f32, dt: f32, limit: f32) -> f32 {
        let hi = if limit < self.out_max {
            limit
        } else {
            self.out_max
        };
        let err = reference - measured;
        let candidate = self.integral + self.ki * err * dt;
        let unclamped = self.kp * err + candidate;
        let out = clamp(unclamped, self.out_min, hi);
        // Anti-windup: only accept the new integral if it does not push further into
        // saturation.
        let saturated_high = unclamped > hi && err > 0.0;
        let saturated_low = unclamped < self.out_min && err < 0.0;
        if !(saturated_high || saturated_low) {
            self.integral = clamp(candidate, self.out_min, self.out_max);
        }
        out
    }

    /// Clear the integrator, e.g. when authority is removed.
    pub fn reset(&mut self) {
        self.integral = 0.0;
    }
}

/// Limits how fast a reference may change.
#[derive(Clone, Copy, Debug)]
pub struct RateLimiter {
    value: f32,
}

impl RateLimiter {
    /// Start at `initial`.
    pub const fn new(initial: f32) -> Self {
        Self { value: initial }
    }

    /// Move toward `target` by at most `max_rate × dt`.
    pub fn step(&mut self, target: f32, max_rate: f32, dt: f32) -> f32 {
        let max_step = max_rate * dt;
        let delta = clamp(target - self.value, -max_step, max_step);
        self.value += delta;
        self.value
    }

    /// Jump to `value` (used to start bumplessly from the measured position).
    pub fn reset(&mut self, value: f32) {
        self.value = value;
    }

    /// Current output.
    pub const fn value(&self) -> f32 {
        self.value
    }
}

/// `f32::clamp` that never panics on inverted bounds and maps NaN to `lo`.
pub fn clamp(v: f32, lo: f32, hi: f32) -> f32 {
    if v > hi {
        hi
    } else if v >= lo {
        v
    } else {
        lo
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clamp_handles_nan() {
        assert_eq!(clamp(f32::NAN, 0.0, 1.0), 0.0);
        assert_eq!(clamp(2.0, 0.0, 1.0), 1.0);
        assert_eq!(clamp(-2.0, 0.0, 1.0), 0.0);
    }

    #[test]
    fn rate_limiter_bounds_step() {
        let mut r = RateLimiter::new(0.0);
        assert!((r.step(1.0, 0.5, 0.1) - 0.05).abs() < 1e-6);
        r.reset(0.9);
        assert!((r.step(0.0, 1.0, 0.1) - 0.8).abs() < 1e-6);
    }

    #[test]
    fn pi_output_respects_limit_and_does_not_wind_up() {
        let mut pi = PiController::new(1.0, 10.0, 0.0, 1.0);
        // Large persistent error with a low authority limit.
        for _ in 0..1000 {
            let u = pi.update(1.0, 0.0, 0.01, 0.4);
            assert!(u <= 0.4 + 1e-6);
        }
        // Once the error reverses, output must come down immediately rather than
        // waiting for a wound-up integrator to unwind.
        let u = pi.update(0.0, 0.5, 0.01, 1.0);
        assert!(u < 0.4, "integrator wound up: u = {u}");
    }

    #[test]
    fn pi_converges_on_first_order_plant() {
        let mut pi = PiController::new(0.8, 8.0, 0.0, 1.0);
        let (mut y, dt, tau) = (0.0f32, 0.01f32, 0.05f32);
        for _ in 0..300 {
            let u = pi.update(0.5, y, dt, 1.0);
            y += (u - y) * dt / tau;
        }
        assert!((y - 0.5).abs() < 0.01, "y = {y}");
    }
}
