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
    /// Maximum engine torque (H*m)
    pub max_torque: f32,
    /// Maximum vehicle linear speed (m/s)
    pub max_speed: f32,

    pub wheel_common: WheelConfig,
    pub wheels: [WheelInstanceConfig; 4],
}

#[derive(Clone, Debug, Deserialize)]
pub struct WheelConfig {
    pub radius: f32,
    pub width: f32,
    /// Mass of wheel (kg)
    pub mass: f32,
    /// Moment of inertia around wheel axis (kg*m^2)
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
