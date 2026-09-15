use crate::{
    config::{VehicleConfig, WheelConfig, WheelInstanceConfig},
    terrain::Terrain,
};
use glam::{Affine3A, Quat, Vec3};
use phy::{Context, Rk4, Rot2, Rot3, Solver, System, Var, Visitor, angular_to_linear3, torque3};
const GRAVITY: Vec3 = Vec3::new(0.0, 0.0, -9.8);
pub struct Vehicle {
    config: VehicleConfig,

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
    /// Fixed angular speed
    fixed_asp: Option<f32>,
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
            fixed_asp: None,
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
        self.visible_asp = self.fixed_asp.unwrap_or(0.0);
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

    fn add_vel_at_poc(&self, outer_vel: Vec3, normal: Vec3) -> Vec3 {
        if let Some(asp) = self.fixed_asp {
            angular_to_linear3(asp * self.axis, -self.common.radius * Vec3::Z)
        } else {
            -outer_vel.project_onto_normalized(self.axis.cross(normal).normalize_or_zero())
        }
    }

    fn set_visible_asp(&mut self, outer_vel: Vec3) {
        if let Some(asp) = self.fixed_asp {
            self.visible_asp = asp;
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

    /// Returns force applied
    fn dry_friction(
        &mut self,
        normal_reaction: Vec3,
        first: bool,
        mut vel_at: Vec3,
        mut acc_at: Vec3,
        eff_mass: f32,
        dt: f32,
    ) -> Vec3 {
        if self.fixed_asp.is_none() {
            let dir = self.axis.cross(normal_reaction).normalize_or_zero();
            vel_at = vel_at.reject_from_normalized(dir);
            acc_at = acc_at.reject_from_normalized(dir);
        }
        let stiction = -eff_mass * (vel_at / dt + acc_at).reject_from(normal_reaction);

        let force_abs = stiction.length();
        let force_abs_max = Terrain::DRY_FRICTION * normal_reaction.length();

        if force_abs < force_abs_max {
            stiction
        } else if first {
            stiction * (force_abs_max / force_abs)
        } else {
            Vec3::ZERO
        }
    }

    const ROLL_FRICTION: f32 = 100.0;

    fn roll_friction(&mut self, normal: Vec3, outer_vel: Vec3) -> Vec3 {
        if self.fixed_asp.is_none() {
            -Self::ROLL_FRICTION
                * outer_vel.project_onto_normalized(self.axis.cross(normal).normalize_or_zero())
        } else {
            Vec3::ZERO
        }
    }
}
impl Vehicle {
    pub fn new(config: VehicleConfig, pos: Vec3, rot: Quat) -> Self {
        Self {
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
        for wheel in &mut self.wheels {
            wheel.fixed_asp = None;
            wheel.axis = Vec3::X;
        }
    }

    /// Vehicle speed (m/s)
    const SPEED: f32 = 6.0;

    pub fn accelerate(&mut self, throttle: f32) {
        for wheel in &mut self.wheels {
            if wheel.config.drive {
                wheel.fixed_asp = Some(-throttle * Self::SPEED / wheel.common.radius);
            }
        }
    }
    pub fn steer(&mut self, angle: f32) {
        // Front wheels
        for wheel in &mut self.wheels[..2] {
            wheel.axis = Vec3::new(angle.cos(), angle.sin(), 0.0);
        }
    }
    pub fn brake(&mut self) {
        for wheel in &mut self.wheels {
            wheel.fixed_asp = Some(0.0);
        }
    }

    fn compute_basic_derivs(&mut self) {
        self.pos.deriv += *self.vel;
        self.rot.deriv += self.rot.transform(*self.rasp);

        self.vel.deriv += GRAVITY;

        let inert = self.config.principal_moments_of_inertia;
        // According to Euler's equation
        self.rasp.deriv += -self.rasp.cross(inert * *self.rasp) / inert;
    }

    fn interact_with_terrain(&mut self, terrain: &Terrain, dt: f32) {
        let map = Affine3A::from_rotation_translation(Quat::from(*self.rot), *self.pos);
        let irot = self.rot.inverse();

        let mut normal_reactions = [None::<Vec3>; 4];
        for (wheel, normal_reaction) in self.wheels.iter_mut().zip(normal_reactions.iter_mut()) {
            if let Some((_poc, normal)) = wheel.contact_terrain(map, terrain) {
                // Use only local coordinates
                let normal = irot.transform(normal);
                let poc = wheel.poc();

                let outer_vel = irot.transform(*self.vel) + angular_to_linear3(*self.rasp, poc);
                wheel.set_visible_asp(outer_vel);
                let vel_at = outer_vel + wheel.add_vel_at_poc(outer_vel, normal);

                let force = wheel.normal_reaction(normal, vel_at);
                if force.length_squared() <= 1e-12 {
                    continue;
                }
                *normal_reaction = Some(force);

                self.vel.deriv += self.rot.transform(force) / self.config.mass;
                self.rasp.deriv += torque3(poc, force) / self.config.principal_moments_of_inertia;
            }
        }

        for i in 0..2 {
            for (wheel, normal_reaction) in self.wheels.iter_mut().zip(normal_reactions) {
                if let Some(normal_reaction) = normal_reaction {
                    // Use only local coordinates
                    let normal = normal_reaction.normalize();
                    let poc = wheel.poc();

                    let outer_vel = irot.transform(*self.vel) + angular_to_linear3(*self.rasp, poc);
                    let vel_at = outer_vel + wheel.add_vel_at_poc(outer_vel, normal);
                    let acc_at =
                        irot.transform(self.vel.deriv) + angular_to_linear3(self.rasp.deriv, poc);

                    let dir = vel_at.reject_from_normalized(normal).normalize_or_zero();
                    let eff_mass = 1.0
                        / (1.0 / self.config.mass
                            + (dir.cross(poc))
                                .dot(dir.cross(poc) / self.config.principal_moments_of_inertia));

                    let mut force =
                        wheel.dry_friction(normal_reaction, i == 0, vel_at, acc_at, eff_mass, dt);
                    if i == 0 {
                        force += wheel.roll_friction(normal, outer_vel);
                    }

                    self.vel.deriv += self.rot.transform(force) / self.config.mass;
                    self.rasp.deriv +=
                        torque3(poc, force) / self.config.principal_moments_of_inertia;
                }
            }
        }
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
    pub fn wheel_transforms(&self) -> [Affine3A; 4] {
        self.wheels.each_ref().map(|wheel| {
            self.transform()
                * Affine3A::from_scale_rotation_translation(
                    Vec3::new(wheel.common.radius, wheel.common.radius, wheel.common.width),
                    Quat::from_rotation_z(wheel.axis.y.atan2(wheel.axis.x))
                        * Quat::from_rotation_y(std::f32::consts::FRAC_PI_2)
                        * Quat::from_rotation_z(wheel.rot.angle()),
                    wheel.center(),
                )
        })
    }
}
#[cfg(test)]
mod tests;
