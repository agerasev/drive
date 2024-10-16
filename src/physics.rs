use std::ops::{Deref, DerefMut};

use crate::algebra::{Angular2, Angular3, Linear, Rot2};
use macroquad::math::{Vec2, Vec3};

pub trait Parameter: Sized + Copy + Default {
    type Derivative: Sized + Copy + Linear;
    fn step(self, d: Self::Derivative, dt: f32) -> Self;
}

/// Independent variable.
#[derive(Clone, Copy, Default, Debug)]
pub struct Var<P: Parameter> {
    value: (P, P),
    deriv: (P::Derivative, P::Derivative),
}

impl<P: Parameter> Var<P> {
    pub fn new(value: P) -> Self {
        Var {
            value: (value, Default::default()),
            deriv: Default::default(),
        }
    }
    pub fn into_value(self) -> P {
        self.value.0
    }
    pub fn value(&self) -> &P {
        &self.value.0
    }
    pub fn value_mut(&mut self) -> &mut P {
        &mut self.value.0
    }

    pub fn add_deriv(&mut self, d: P::Derivative) {
        self.deriv.0 += d;
    }
    pub fn deriv(&self) -> &P::Derivative {
        &self.deriv.0
    }
    pub fn deriv_mut(&mut self) -> &mut P::Derivative {
        &mut self.deriv.0
    }
}

impl<P: Parameter> Deref for Var<P> {
    type Target = P;
    fn deref(&self) -> &Self::Target {
        self.value()
    }
}
impl<P: Parameter> DerefMut for Var<P> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.value_mut()
    }
}

pub trait Visitor {
    fn apply<P: Parameter>(&mut self, wp: &mut Var<P>);
}

pub trait System {
    fn compute_derivs(&mut self);
    fn visit_vars<V: Visitor>(&mut self, visitor: &mut V);
}

impl Parameter for f32 {
    type Derivative = f32;
    fn step(self, dp: f32, dt: f32) -> Self {
        self + dp * dt
    }
}
impl Parameter for Vec2 {
    type Derivative = Vec2;
    fn step(self, dp: Vec2, dt: f32) -> Self {
        self + dp * dt
    }
}
impl Parameter for Vec3 {
    type Derivative = Vec3;
    fn step(self, dp: Vec3, dt: f32) -> Self {
        self + dp * dt
    }
}

impl Parameter for Angular2 {
    type Derivative = Angular2;
    fn step(self, dp: Angular2, dt: f32) -> Self {
        self + dp * dt
    }
}
impl Parameter for Angular3 {
    type Derivative = Angular3;
    fn step(self, dp: Angular3, dt: f32) -> Self {
        self + dp * dt
    }
}

impl Parameter for Rot2 {
    type Derivative = Angular2;
    fn step(self, dp: Angular2, dt: f32) -> Self {
        self * (dp * dt).rot()
    }
}

pub struct Solver;

struct Rk4Step {
    stage: u32,
    dt: f32,
}

impl Visitor for Rk4Step {
    fn apply<P: Parameter>(&mut self, v: &mut Var<P>) {
        let p = &mut v.value;
        let d = &mut v.deriv;
        let dt = self.dt;

        match self.stage {
            0 => {
                (p.1, d.1) = (p.0, d.0);
                p.0 = p.1.step(d.0, dt / 2.0);
            }
            1 => {
                d.1 += d.0 * 2.0;
                p.0 = p.1.step(d.0, dt / 2.0);
            }
            2 => {
                d.1 += d.0 * 2.0;
                p.0 = p.1.step(d.0, dt);
            }
            3 => {
                d.1 += d.0;
                p.0 = p.1.step(d.1, dt / 6.0);
            }
            _ => unreachable!(),
        };

        d.0 = Default::default();
    }
}

impl Solver {
    pub fn solve_step<S: System>(&self, system: &mut S, dt: f32) {
        for stage in 0..4 {
            system.compute_derivs();
            system.visit_vars(&mut Rk4Step { stage, dt });
        }
    }
}
