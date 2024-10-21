use derive_more::derive::{From, Into};
use macroquad::math::{Mat2, Mat3, Quat, Vec2, Vec3};
use std::f32::consts::PI;

/// 2D Rotation.
#[derive(Clone, Copy, Default, Debug)]
pub struct Rot2(f32);

/// 3D Rotation.
#[derive(Clone, Copy, Debug, From, Into)]
pub struct Rot3(
    #[from]
    #[into]
    Quat,
);

impl Default for Rot3 {
    fn default() -> Self {
        Self(Quat::IDENTITY)
    }
}

impl Rot2 {
    /// From angle in radians
    pub fn from_angle(angle: f32) -> Self {
        Self(angle % (2.0 * PI))
    }

    /// Angle in radians `0.0..(2.0 * PI)`
    pub fn angle(self) -> f32 {
        self.0
    }
    /// Angle in degrees `0.0..360.0`
    pub fn angle_degrees(self) -> f32 {
        (180.0 / PI) * self.angle()
    }
    pub fn matrix(self) -> Mat2 {
        Mat2::from_angle(self.0)
    }

    pub fn transform(&self, v: Vec2) -> Vec2 {
        self.matrix().mul_vec2(v)
    }
    pub fn chain(self, other: Self) -> Self {
        Self((self.0 + other.0) % (2.0 * PI))
    }
    pub fn inverse(self) -> Self {
        Self(-self.0)
    }
}

impl Rot3 {
    pub fn from_scaled_axis(v: Vec3) -> Self {
        Self(Quat::from_scaled_axis(v))
    }

    pub fn matrix(self) -> Mat3 {
        Mat3::from_quat(self.0)
    }

    pub fn transform(self, v: Vec3) -> Vec3 {
        self.0.mul_vec3(v)
    }
    pub fn chain(self, other: Self) -> Self {
        Self(other.0.mul_quat(self.0).normalize())
    }
    pub fn inverse(self) -> Self {
        Self(self.0.inverse())
    }
}
