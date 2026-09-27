use crate::config::VehicleConfig;

/// Load-proportional rolling resistance, smoothed near rest by the contact model.
pub(super) const ROLL_RESISTANCE: f32 = 0.015;

/// An ideal automatic drivetrain: launch torque is capped, then power is capped.
pub(super) struct Drivetrain {
    launch_force: f32,
    power: f32,
    drag: f32,
    reverse_speed: f32,
}

impl Drivetrain {
    pub fn new(config: &VehicleConfig) -> Self {
        assert!(config.drive_ratio.is_finite() && config.drive_ratio > 0.0);
        assert!(config.max_speed.is_finite() && config.max_speed > 0.0);
        assert!(config.max_reverse_speed.is_finite() && config.max_reverse_speed > 0.0);
        let launch_force = config.max_torque * config.drive_ratio / config.wheel_common.radius;
        let force_at_max_speed = launch_force.min(config.max_power / config.max_speed);
        let rolling_force = ROLL_RESISTANCE * config.mass * super::GRAVITY.length();
        Self {
            launch_force,
            power: config.max_power,
            drag: (force_at_max_speed - rolling_force).max(0.0) / config.max_speed.powi(2),
            reverse_speed: config.max_reverse_speed,
        }
    }

    /// Total drive force before tire limits. Divide both torque and power among
    /// driven wheels; evaluate using each wheel's longitudinal contact speed.
    pub fn force(&self, speed: f32, throttle: f32) -> f32 {
        let force = self.launch_force.min(self.power / speed.abs().max(0.1));
        let limiter = if throttle < 0.0 {
            ((self.reverse_speed + speed) / (0.2 * self.reverse_speed)).clamp(0.0, 1.0)
        } else {
            1.0
        };
        throttle * force * limiter
    }

    pub fn air_resistance(&self, speed: f32) -> f32 {
        -self.drag * speed * speed.abs()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> VehicleConfig {
        serde_json::from_slice(include_bytes!("../../assets/logan/config.json")).unwrap()
    }

    #[test]
    fn effort_is_torque_limited_at_rest_and_power_limited_at_speed() {
        let config = config();
        let drive = Drivetrain::new(&config);
        let launch = drive.force(0.0, 1.0);
        assert!(launch.is_finite() && launch > 0.0);
        assert_eq!(drive.force(0.0, 0.25), launch * 0.25);
        assert_eq!(drive.force(0.0, -1.0), -launch);
        for speed in [20.0, 40.0, 60.0] {
            assert!(drive.force(speed, 1.0) < launch);
            assert!((drive.force(speed, 1.0) * speed - config.max_power).abs() < 0.01);
            assert_eq!(drive.force(speed, 0.25), drive.force(speed, 1.0) * 0.25);
        }
    }

    #[test]
    fn gearing_changes_launch_force_without_multiplying_engine_power() {
        let mut config = config();
        let original = Drivetrain::new(&config);
        config.drive_ratio *= 2.0;
        let shorter = Drivetrain::new(&config);
        assert_eq!(shorter.force(0.0, 1.0), 2.0 * original.force(0.0, 1.0));
        assert_eq!(shorter.force(40.0, 1.0), original.force(40.0, 1.0));
    }

    #[test]
    fn reverse_cutout_is_independent_of_throttle_and_drag_always_opposes_motion() {
        let config = config();
        let drive = Drivetrain::new(&config);
        for throttle in [-1.0, -0.25] {
            assert!(drive.force(-0.5 * config.max_reverse_speed, throttle) < 0.0);
            assert_eq!(drive.force(-config.max_reverse_speed, throttle), 0.0);
            assert_eq!(drive.force(-2.0 * config.max_reverse_speed, throttle), 0.0);
        }
        for speed in [-40.0, -10.0, 10.0, 40.0] {
            assert!(drive.air_resistance(speed) * speed < 0.0);
        }
        assert_eq!(drive.air_resistance(0.0), 0.0);
    }
}
