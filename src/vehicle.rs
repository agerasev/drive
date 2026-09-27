//! Contact-force driving with automatic torque/power limits and traction control.
//!
//! Throttle requests effort, not wheel speed. Each driven wheel receives an equal
//! share of engine capacity, limited by its suspension load and the grip left
//! after cornering. Excess throttle does not create artificial longitudinal slip.
//! Lateral slip relaxes over a fixed response time rather than one physics step,
//! allowing the parallel front wheels to turn without exhausting their grip.
//! Rolling resistance acts with or without throttle; longitudinal aerodynamic drag
//! is calibrated from the configured level-ground speed. Reverse has a separate
//! propulsion limit, and opposite-direction throttle brakes before reversing.
//!
//! Wheel spin follows ground motion (or locks under braking); wheel inertia,
//! burnouts, discrete gears and engine RPM are not simulated. Controls persist
//! across physics steps until replaced or reset.
//!
//! ```
//! # fn drive(car: &mut drive::vehicle::Vehicle, terrain: &drive::terrain::Terrain) {
//! car.reset_controls();
//! car.accelerate(0.5); // Half of the available engine effort.
//! car.step(terrain, 1.0 / 240.0);
//! # }
//! ```

use crate::{
    config::{VehicleConfig, WheelConfig, WheelInstanceConfig},
    terrain::Terrain,
};
use glam::{Affine3A, Mat2, Quat, Vec2, Vec3};
use phy::{Context, Rk4, Rot2, Rot3, Solver, System, Var, Visitor, angular_to_linear3, torque3};

mod drivetrain;
use drivetrain::{Drivetrain, ROLL_RESISTANCE};

const GRAVITY: Vec3 = Vec3::new(0.0, 0.0, -9.8);
// Parallel steering axes cannot both roll without lateral slip in a turn.
// A finite relaxation time avoids spending all grip on that small mismatch and
// keeps the lateral response independent of the physics timestep.
const LATERAL_RELAXATION: f32 = 0.08;

#[derive(Clone, Copy, Default)]
enum Control {
    #[default]
    Coast,
    Throttle(f32),
    Brake,
}

pub struct Vehicle {
    config: VehicleConfig,
    drivetrain: Drivetrain,
    control: Control,

    pos: Var<Vec3, Rk4>,
    rot: Var<Rot3, Rk4>,

    vel: Var<Vec3, Rk4>,
    /// Angular speed in rotating reference frame around principal axes of inertia coordinates
    rasp: Var<Vec3, Rk4>,

    wheels: [Wheel; 4],
}

struct Wheel {
    common: WheelConfig,
    config: WheelInstanceConfig,

    axis: Vec3,
    rot: Var<Rot2, Rk4>,
    /// Visible angular speed
    visible_asp: f32,

    dev: f32,
}

impl Wheel {
    fn new(common: WheelConfig, config: WheelInstanceConfig) -> Self {
        Self {
            common,
            config,
            axis: Vec3::X,
            rot: Var::default(),
            visible_asp: 0.0,
            dev: 0.0,
        }
    }

    pub fn center(&self) -> Vec3 {
        self.config.center + self.dev * Vec3::Z
    }
    pub fn poc(&self) -> Vec3 {
        self.center() - self.common.radius * Vec3::Z
    }
    pub fn lower_poc(&self) -> Vec3 {
        self.config.center - self.common.radius * Vec3::Z
    }
    pub fn upper_poc(&self) -> Vec3 {
        self.config.center + (self.common.travel - self.common.radius) * Vec3::Z
    }

    /// Returns point of contact and normal
    fn contact_terrain(&mut self, map: Affine3A, terrain: &Terrain) -> Option<(Vec3, Vec3)> {
        self.dev = 0.0;
        terrain
            .intersect_line(
                map.transform_point3(self.upper_poc()),
                map.transform_point3(self.lower_poc()),
            )
            .map(|(dist, poc, normal)| {
                self.dev = self.common.travel - dist;
                (poc, normal)
            })
    }

    fn set_visible_asp(&mut self, outer_vel: Vec3, braking: bool) {
        if braking {
            self.visible_asp = 0.0;
        } else {
            let r = -self.common.radius * Vec3::Z;
            self.visible_asp = outer_vel.cross(r).dot(self.axis) / self.common.radius.powi(2);
        }
    }

    /// Returns force applied
    fn normal_reaction(&mut self, normal: Vec3, vel_at: Vec3) -> Vec3 {
        let susp = self.dev * (self.common.hardness.0 + self.dev * self.common.hardness.1)
            - vel_at.dot(Vec3::Z) * self.common.damping;

        (susp.max(0.0) * Vec3::Z).project_onto_normalized(normal)
    }
}
impl Vehicle {
    pub fn new(config: VehicleConfig, pos: Vec3, rot: Quat) -> Self {
        Self {
            drivetrain: Drivetrain::new(&config),
            control: Control::default(),
            wheels: config
                .wheels
                .clone()
                .map(|wc| Wheel::new(config.wheel_common.clone(), wc)),
            config,
            pos: Var::new(pos),
            rot: Var::new(Rot3::from(rot)),
            vel: Var::default(),
            rasp: Var::default(),
        }
    }

    pub fn pos(&self) -> Vec3 {
        *self.pos
    }

    pub fn reset_controls(&mut self) {
        self.control = Control::Coast;
        for wheel in &mut self.wheels {
            wheel.axis = Vec3::X;
        }
    }

    /// Request signed engine effort in [-1, 1]; zero coasts, negative reverses.
    /// Opposite-direction input brakes until longitudinal speed is below 0.2 m/s.
    pub fn accelerate(&mut self, throttle: f32) {
        assert!(throttle.is_finite());
        self.control = Control::Throttle(throttle.clamp(-1.0, 1.0));
    }
    pub fn steer(&mut self, angle: f32) {
        // Front wheels
        for wheel in &mut self.wheels[..2] {
            wheel.axis = Vec3::new(angle.cos(), angle.sin(), 0.0);
        }
    }
    pub fn brake(&mut self) {
        self.control = Control::Brake;
    }

    fn pedals(&self) -> (f32, bool) {
        match self.control {
            Control::Coast => (0.0, false),
            Control::Brake => (0.0, true),
            Control::Throttle(throttle) => {
                let speed = self.rot.inverse().transform(*self.vel).y;
                let braking = throttle != 0.0 && throttle.signum() * speed < -0.2;
                (if braking { 0.0 } else { throttle }, braking)
            }
        }
    }

    fn compute_basic_derivs(&mut self) {
        self.pos.deriv += *self.vel;
        self.rot.deriv += self.rot.transform(*self.rasp);

        self.vel.deriv += GRAVITY;
        let speed = self.rot.inverse().transform(*self.vel).y;
        self.vel.deriv += self.rot.transform(Vec3::Y)
            * (self.drivetrain.air_resistance(speed) / self.config.mass);

        let inert = self.config.principal_moments_of_inertia;
        // According to Euler's equation
        self.rasp.deriv += -self.rasp.cross(inert * *self.rasp) / inert;
    }

    fn interact_with_terrain(&mut self, terrain: &Terrain, dt: f32) {
        let map = Affine3A::from_rotation_translation(Quat::from(*self.rot), *self.pos);
        let irot = self.rot.inverse();
        let (throttle, braking) = self.pedals();
        let driven_wheels = self
            .wheels
            .iter()
            .filter(|wheel| wheel.config.drive)
            .count()
            .max(1) as f32;

        let mut normal_reactions = [None::<Vec3>; 4];
        for (wheel, normal_reaction) in self.wheels.iter_mut().zip(normal_reactions.iter_mut()) {
            wheel.visible_asp = 0.0;
            if let Some((_poc, normal)) = wheel.contact_terrain(map, terrain) {
                // Use only local coordinates
                let normal = irot.transform(normal);
                let poc = wheel.poc();

                let outer_vel = irot.transform(*self.vel) + angular_to_linear3(*self.rasp, poc);
                wheel.set_visible_asp(outer_vel, braking);

                let force = wheel.normal_reaction(normal, outer_vel);
                if force.length_squared() <= 1e-12 {
                    continue;
                }
                *normal_reaction = Some(force);

                self.vel.deriv += self.rot.transform(force) / self.config.mass;
                self.rasp.deriv += torque3(poc, force) / self.config.principal_moments_of_inertia;
            }
        }

        // Braking solves coupled contact constraints. Track total force per tire
        // so repeated corrections cannot exceed its friction circle. Alternating
        // sweeps reduce wheel-order bias; rolling tires need only one force pass.
        let mut forces = [Vec3::ZERO; 4];
        for pass in 0..if braking { 8 } else { 1 } {
            for index in 0..4 {
                let index = if pass % 2 == 0 { index } else { 3 - index };
                let Some(reaction) = normal_reactions[index] else {
                    continue;
                };
                let wheel = &self.wheels[index];
                let normal = reaction.normalize();
                let forward = normal.cross(wheel.axis).normalize_or_zero();
                let lateral = forward.cross(normal);
                let poc = wheel.poc();
                let vel_at = irot.transform(*self.vel) + angular_to_linear3(*self.rasp, poc);
                let speed = vel_at.dot(forward);
                let limit = Terrain::DRY_FRICTION * reaction.length();

                let force = if braking {
                    // Solve both tangent directions together; independently
                    // clamping them can sacrifice braking to a tiny side slip.
                    let response = self.tangent_inverse_mass(poc, forward, lateral);
                    let acc = vel_at / dt + self.acceleration_at(poc);
                    let correction =
                        -response.inverse() * Vec2::new(acc.dot(forward), acc.dot(lateral));
                    (forces[index] + forward * correction.x + lateral * correction.y)
                        .clamp_length_max(limit)
                } else {
                    let drive = if wheel.config.drive {
                        self.drivetrain.force(speed, throttle) / driven_wheels
                    } else {
                        0.0
                    };
                    let longitudinal = drive
                        - ROLL_RESISTANCE * reaction.length() * (speed / 0.5).clamp(-1.0, 1.0);
                    let sideways = -self.effective_mass(poc, lateral) * vel_at.dot(lateral)
                        / dt.max(LATERAL_RELAXATION);
                    let sideways = sideways.clamp(-limit, limit);
                    let traction = (limit * limit - sideways * sideways).max(0.0).sqrt();
                    lateral * sideways + forward * longitudinal.clamp(-traction, traction)
                };
                self.apply_force(poc, force - forces[index]);
                forces[index] = force;
            }
        }
    }

    fn apply_force(&mut self, poc: Vec3, force: Vec3) {
        self.vel.deriv += self.rot.transform(force) / self.config.mass;
        self.rasp.deriv += torque3(poc, force) / self.config.principal_moments_of_inertia;
    }

    fn effective_mass(&self, poc: Vec3, dir: Vec3) -> f32 {
        let arm = poc.cross(dir);
        1.0 / (1.0 / self.config.mass + arm.dot(arm / self.config.principal_moments_of_inertia))
    }

    fn tangent_inverse_mass(&self, poc: Vec3, forward: Vec3, lateral: Vec3) -> Mat2 {
        let front_arm = poc.cross(forward);
        let side_arm = poc.cross(lateral);
        let coupling = front_arm.dot(side_arm / self.config.principal_moments_of_inertia);
        Mat2::from_cols(
            Vec2::new(1.0 / self.effective_mass(poc, forward), coupling),
            Vec2::new(coupling, 1.0 / self.effective_mass(poc, lateral)),
        )
    }

    fn acceleration_at(&self, poc: Vec3) -> Vec3 {
        self.rot.inverse().transform(self.vel.deriv)
            + angular_to_linear3(self.rasp.deriv, poc)
            + self.rasp.cross(self.rasp.cross(poc))
    }

    fn visit_vars<V: Visitor<Rk4>>(&mut self, visitor: &mut V) {
        visitor.apply(&mut self.pos);
        visitor.apply(&mut self.rot);
        visitor.apply(&mut self.vel);
        visitor.apply(&mut self.rasp);
        for wheel in &mut self.wheels {
            visitor.apply(&mut wheel.rot);
        }
    }
}

struct World<'a>(&'a Terrain, &'a mut Vehicle);
impl System<Rk4> for World<'_> {
    fn compute_derivs(&mut self, ctx: &<Rk4 as Solver>::Context) {
        self.1.compute_basic_derivs();
        self.1.interact_with_terrain(self.0, ctx.time_step());
        for wheel in &mut self.1.wheels {
            wheel.rot.deriv += wheel.visible_asp;
        }
    }
    fn visit_vars<V: Visitor<Rk4>>(&mut self, visitor: &mut V) {
        self.1.visit_vars(visitor);
    }
}
impl Vehicle {
    /// A positive fixed step; zero is a no-op because static friction divides by dt.
    pub fn step(&mut self, terrain: &Terrain, dt: f32) {
        if dt == 0.0 {
            return;
        }
        assert!(dt.is_finite() && dt > 0.0);
        Rk4.solve_step(&mut World(terrain, self), dt);
        // RK4's last force evaluation is not the final integrated pose.
        // Refresh contact-derived values for drawing without accumulating forces.
        let map = self.transform();
        for wheel in &mut self.wheels {
            wheel.contact_terrain(map, terrain);
        }
    }
    pub fn transform(&self) -> Affine3A {
        Affine3A::from_rotation_translation(Quat::from(*self.rot), *self.pos)
    }
    pub fn velocity(&self) -> Vec3 {
        *self.vel
    }
    /// Wheel-local +Z points outward; the sole spoke face is on that side.
    pub fn wheel_transforms(&self) -> [Affine3A; 4] {
        self.wheels.each_ref().map(|wheel| {
            self.transform()
                * Affine3A::from_scale_rotation_translation(
                    Vec3::new(wheel.common.radius, wheel.common.radius, wheel.common.width),
                    Quat::from_rotation_z(wheel.axis.y.atan2(wheel.axis.x))
                        * Quat::from_rotation_y(std::f32::consts::FRAC_PI_2)
                        * Quat::from_rotation_z(wheel.rot.angle())
                        * if wheel.config.center.x < 0.0 {
                            Quat::from_rotation_y(std::f32::consts::PI)
                        } else {
                            Quat::IDENTITY
                        },
                    wheel.center(),
                )
        })
    }
}
#[cfg(test)]
mod tests;
