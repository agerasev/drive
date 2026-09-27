use glam::Vec3;
use serde::Deserialize;
#[derive(Clone, Debug, Deserialize)]
pub struct VehicleConfig {
    /// Mass (kg)
    pub mass: f32,
    /// Principal moments of inertia (kg*m^2)
    pub principal_moments_of_inertia: Vec3,

    /// Maximum engine power (W)
    pub max_power: f32,
    /// Maximum engine torque (N*m), before gearing and distribution to driven wheels.
    pub max_torque: f32,
    /// Full-throttle level-ground speed (m/s), used to calibrate aerodynamic drag.
    /// Limited traction or insufficient launch torque can produce a lower speed.
    pub max_speed: f32,
    /// Effective launch reduction; torque is multiplied by this ratio.
    #[serde(default = "default_drive_ratio")]
    pub drive_ratio: f32,
    /// Reverse propulsion cuts out at this speed (m/s), independently of throttle.
    #[serde(default = "default_reverse_speed")]
    pub max_reverse_speed: f32,

    pub wheel_common: WheelConfig,
    pub wheels: [WheelInstanceConfig; 4],
}

fn default_drive_ratio() -> f32 {
    10.0
}

fn default_reverse_speed() -> f32 {
    6.0
}

#[derive(Clone, Debug, Deserialize)]
pub struct WheelConfig {
    pub radius: f32,
    pub width: f32,
    /// Mass of wheel (kg); not simulated separately from the chassis.
    pub mass: f32,
    /// Moment of inertia around wheel axis (kg*m^2); not currently integrated.
    pub moment_of_inertia: f32,

    /// Spring linear and quadratic hardness (N/m, N/m^2)
    pub hardness: (f32, f32),
    /// Liquid friction of shock absorber (N/(m/s))
    pub damping: f32,

    /// Maximum wheel deviation from lower position to upper position
    pub travel: f32,
}

#[derive(Clone, Debug, Deserialize)]
pub struct WheelInstanceConfig {
    /// Position of equilibrium (when no force apllied, lower postion)
    pub center: Vec3,
    /// Does engine moment is transmitted to this wheel
    pub drive: bool,
}
