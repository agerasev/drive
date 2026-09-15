use std::time::Duration;
pub const STEP: Duration = Duration::from_nanos(4_166_667);
const MAX_STEPS: u32 = 24;
#[derive(Default)]
pub struct Clock {
    accumulated: Duration,
}
impl Clock {
    pub fn steps(&mut self, elapsed: Duration, active: bool, slow: bool) -> u32 {
        if !active {
            self.accumulated = Duration::ZERO;
            return 0;
        }
        let elapsed = elapsed.min(STEP * MAX_STEPS);
        self.accumulated += if slow { elapsed / 2 } else { elapsed };
        let count = (self.accumulated.as_nanos() / STEP.as_nanos()) as u32;
        self.accumulated -= STEP * count;
        count
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn frame_partition_does_not_change_simulated_time_and_stalls_are_bounded() {
        let mut a = Clock::default();
        let mut b = Clock::default();
        let n: u32 = (0..100).map(|_| a.steps(STEP * 2, true, false)).sum();
        let m: u32 = (0..50).map(|_| b.steps(STEP * 4, true, false)).sum();
        assert_eq!(n, m);
        assert_eq!(a.steps(Duration::from_secs(10), true, false), MAX_STEPS);
        assert_eq!(a.steps(Duration::from_secs(10), false, false), 0);
        assert_eq!(a.steps(STEP * 2, true, true), 1);
    }
}
