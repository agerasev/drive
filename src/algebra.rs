use derive_more::derive::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};
use macroquad::math::{Mat2, Vec2, Vec3};
use std::{
    f32::consts::PI,
    ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub, SubAssign},
};

pub trait Linear:
    Default
    + Add<Output = Self>
    + AddAssign
    + Sub<Output = Self>
    + SubAssign
    + Mul<f32, Output = Self>
    + MulAssign<f32>
    + Div<f32, Output = Self>
    + DivAssign<f32>
{
}

impl<T> Linear for T where
    T: Default
        + Add<Output = Self>
        + AddAssign
        + Sub<Output = Self>
        + SubAssign
        + Mul<f32, Output = Self>
        + MulAssign<f32>
        + Div<f32, Output = Self>
        + DivAssign<f32>
{
}

#[derive(
    Clone,
    Copy,
    PartialEq,
    PartialOrd,
    Default,
    Debug,
    Neg,
    Add,
    AddAssign,
    Sub,
    SubAssign,
    Mul,
    MulAssign,
    Div,
    DivAssign,
)]
pub struct Angular2(pub f32);

#[derive(
    Clone,
    Copy,
    PartialEq,
    Default,
    Debug,
    Neg,
    Add,
    AddAssign,
    Sub,
    SubAssign,
    Mul,
    MulAssign,
    Div,
    DivAssign,
)]
pub struct Angular3(pub Vec3);

/// 2D Rotation.
#[derive(Clone, Copy, Default, Debug)]
pub struct Rot2(f32);

/*
/// 3D Rotation.
#[derive(Clone, Copy, Debug)]
pub struct Rot3(Quat);

impl Default for Rot3 {
    fn default() -> Self {
        Self(Quat::IDENTITY)
    }
}
*/

impl Rot2 {
    /// Angle in radians `0.0..(2.0 * PI)`
    pub fn angle(self) -> f32 {
        self.0
    }
    /// Angle in degrees `0.0..360.0`
    pub fn angle_degrees(self) -> f32 {
        180.0 / PI * self.angle()
    }

    pub fn matrix(self) -> Mat2 {
        Mat2::from_angle(self.0)
    }
    pub fn apply(self, v: Vec2) -> Vec2 {
        self.matrix().mul_vec2(v)
    }
    pub fn chain(self, other: Self) -> Self {
        Self(self.0 + other.0)
    }
}

impl Mul for Rot2 {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self::Output {
        self.chain(rhs)
    }
}

impl Mul<f32> for Rot2 {
    type Output = Self;
    fn mul(self, rhs: f32) -> Self::Output {
        Self(self.0 * rhs)
    }
}

impl Angular2 {
    pub fn rot(self) -> Rot2 {
        Rot2(self.0)
    }
    pub fn vel_at(self, r: Vec2) -> Vec2 {
        self.0 * r.perp()
    }
    pub fn torque(r: Vec2, f: Vec2) -> Self {
        Self(r.perp_dot(f))
    }
}
