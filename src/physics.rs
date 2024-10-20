use crate::{
    algebra::{Rot2, Rot3},
    numerical::Parameter,
};
use macroquad::math::{Vec2, Vec3};

impl Parameter for Rot2 {
    /// Angular speed
    type Derivative = f32;
    fn step(self, dp: f32, dt: f32) -> Self {
        self.chain(Rot2::from_angle(dp * dt))
    }
}
impl Parameter for Rot3 {
    /// Angular speed around axes
    type Derivative = Vec3;
    fn step(self, dp: Vec3, dt: f32) -> Self {
        self.chain(Rot3::from_scaled_axis(dp * dt))
    }
}

pub fn torque2(pos: Vec2, vec: Vec2) -> f32 {
    pos.perp_dot(vec)
}
pub fn torque3(pos: Vec3, vec: Vec3) -> Vec3 {
    pos.cross(vec)
}

pub fn angular_to_linear2(angular: f32, pos: Vec2) -> Vec2 {
    angular * pos.perp()
}
pub fn angular_to_linear3(angular: Vec3, pos: Vec3) -> Vec3 {
    angular.cross(pos)
}
