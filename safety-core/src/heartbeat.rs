//! Software heartbeat supervision of cooperative tasks.
//!
//! In a cooperative async executor a task that awaits something that never completes
//! does not crash; it just goes quiet. Each supervised task therefore reports a
//! heartbeat, and the supervisor only feeds the hardware watchdog when *every* task
//! has reported within its deadline. A task that blocks the executor entirely also
//! stops the supervisor, so the hardware watchdog covers that case.

/// Tracks the last heartbeat of `N` tasks, each with its own timeout.
#[derive(Clone, Copy, Debug)]
pub struct HeartbeatMonitor<const N: usize> {
    last_ms: [u64; N],
    timeout_ms: [u64; N],
}

impl<const N: usize> HeartbeatMonitor<N> {
    /// All tasks are considered alive at `now_ms`, giving each one full timeout to
    /// report for the first time.
    pub const fn new(timeout_ms: [u64; N], now_ms: u64) -> Self {
        Self {
            last_ms: [now_ms; N],
            timeout_ms,
        }
    }

    /// Record a heartbeat. Out-of-range ids are ignored.
    pub fn kick(&mut self, task: usize, now_ms: u64) {
        if let Some(t) = self.last_ms.get_mut(task) {
            *t = now_ms;
        }
    }

    /// `Ok` if every task is within its deadline, otherwise the first late task id.
    pub fn check(&self, now_ms: u64) -> Result<(), usize> {
        for i in 0..N {
            if now_ms.saturating_sub(self.last_ms[i]) > self.timeout_ms[i] {
                return Err(i);
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detects_the_late_task() {
        let mut hb = HeartbeatMonitor::new([50, 200], 0);
        assert_eq!(hb.check(50), Ok(()));
        hb.kick(0, 40);
        hb.kick(1, 40);
        assert_eq!(hb.check(90), Ok(()));
        assert_eq!(hb.check(91), Err(0));
        hb.kick(0, 91);
        assert_eq!(hb.check(140), Ok(()));
        // Task 0 keeps reporting; task 1 last reported at 40 with a 200 ms timeout.
        hb.kick(0, 230);
        assert_eq!(hb.check(240), Ok(()));
        assert_eq!(hb.check(241), Err(1));
    }

    #[test]
    fn ignores_unknown_ids() {
        let mut hb = HeartbeatMonitor::new([10], 0);
        hb.kick(5, 100);
        assert_eq!(hb.check(11), Err(0));
    }
}
