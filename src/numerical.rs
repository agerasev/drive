use macroquad::math::{Vec2, Vec3};
use std::ops::{Add, Deref, DerefMut, Mul};

pub trait Parameter: Sized + Copy + Default {
    type Derivative: Sized
        + Copy
        + Add<Output = Self::Derivative>
        + Mul<f32, Output = Self::Derivative>
        + Default;
    fn step(self, deriv: Self::Derivative, dt: f32) -> Self;
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
        self.deriv.0 = self.deriv.0 + d;
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
    fn apply<P: Parameter>(&mut self, v: &mut Var<P>);
}

pub trait System {
    fn compute_derivs(&mut self, dt: f32);
    fn visit_vars<V: Visitor>(&mut self, visitor: &mut V);
}

impl Parameter for f32 {
    type Derivative = f32;
    fn step(self, deriv: f32, dt: f32) -> Self {
        self + deriv * dt
    }
}
impl Parameter for Vec2 {
    type Derivative = Vec2;
    fn step(self, deriv: Vec2, dt: f32) -> Self {
        self + deriv * dt
    }
}
impl Parameter for Vec3 {
    type Derivative = Vec3;
    fn step(self, deriv: Vec3, dt: f32) -> Self {
        self + deriv * dt
    }
}

/// Temporal differential equation solver.
///
/// Currently uses the Runge–Kutta method (RK4).
pub struct Solver;

struct Rk4Step {
    stage: u32,
    dt: f32,
}

impl Rk4Step {
    fn dt(&self) -> f32 {
        match self.stage {
            0 => self.dt / 2.0,
            1 => self.dt / 2.0,
            2 => self.dt,
            3 => self.dt,
            _ => unreachable!(),
        }
    }
}

impl Visitor for Rk4Step {
    fn apply<P: Parameter>(&mut self, p: &mut Var<P>) {
        let x = &mut p.value;
        let dx = &mut p.deriv;
        let dt = self.dt();

        match self.stage {
            0 => {
                (x.1, dx.1) = (x.0, dx.0);
                x.0 = x.1.step(dx.0, dt);
            }
            1 => {
                dx.1 = dx.1 + dx.0 * 2.0;
                x.0 = x.1.step(dx.0, dt);
            }
            2 => {
                dx.1 = dx.1 + dx.0 * 2.0;
                x.0 = x.1.step(dx.0, dt);
            }
            3 => {
                dx.1 = (dx.1 + dx.0) * (1.0 / 6.0);
                x.0 = x.1.step(dx.1, dt);
            }
            _ => unreachable!(),
        };

        dx.0 = Default::default();
    }
}

impl Solver {
    pub fn solve_step<S: System>(&self, system: &mut S, dt: f32) {
        for stage in 0..4 {
            let mut step = Rk4Step { stage, dt };
            system.compute_derivs(step.dt());
            system.visit_vars(&mut step);
        }
    }
}
