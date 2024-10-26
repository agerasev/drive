use super::terrain::Terrain;
use crate::{
    algebra::{Rot2, Rot3},
    model::TransformStack,
    numerical::{Var, Visitor},
    physics::{angular_to_linear3, torque3},
};
use macroquad::{
    math::{Affine3A, Quat, Vec2, Vec3, Vec4},
    models::{draw_mesh, Mesh},
    texture::Texture2D,
    ui::Vertex,
};
use serde::{Deserialize, Deserializer};
use std::{f32::consts::PI, rc::Rc};

const GRAVITY: Vec3 = Vec3::new(0.0, 0.0, -9.8);

fn de_vec2<'de, D: Deserializer<'de>>(deserializer: D) -> Result<Vec2, D::Error> {
    Ok(Vec2::from(<[f32; 2]>::deserialize(deserializer)?))
}
fn de_vec3<'de, D: Deserializer<'de>>(deserializer: D) -> Result<Vec3, D::Error> {
    Ok(Vec3::from(<[f32; 3]>::deserialize(deserializer)?))
}

#[derive(Clone, Debug, Deserialize)]
pub struct VehicleConfig {
    /// Mass (kg)
    pub mass: f32,
    /// Principal moments of inertia (kg*m^2)
    #[serde(deserialize_with = "de_vec3")]
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

    #[serde(deserialize_with = "de_vec2")]
    pub texture_center: Vec2,
    pub texture_radius: f32,
}

#[derive(Clone, Debug, Deserialize)]
pub struct WheelInstanceConfig {
    /// Position of equilibrium (when no force apllied, lower postion)
    #[serde(deserialize_with = "de_vec3")]
    pub center: Vec3,
}

pub fn make_wheel_model(
    n_vertices: usize,
    tex_pos: Vec2,
    tex_rad: f32,
    texture: Texture2D,
) -> Mesh {
    let uv_pos = tex_pos / texture.size();
    let uv_rad = Vec2::splat(tex_rad) / texture.size();
    Mesh {
        vertices: [1.0, -1.0]
            .into_iter()
            .map(|side| Vertex {
                position: Vec3::new(0.0, 0.0, 0.5 * side),
                uv: uv_pos,
                normal: Vec4::new(0.0, 0.0, side, 1.0),
                color: [255; 4],
            })
            .chain((0..n_vertices).flat_map(|i| {
                let phi = 2.0 * PI * (i as f32 / n_vertices as f32);
                let offset = Vec2::from_angle(phi);
                [
                    Vertex {
                        position: Vec3::from((offset, 0.5)),
                        uv: uv_pos + uv_rad * offset,
                        normal: Vec4::new(0.0, 0.0, 1.0, 1.0),
                        color: [255; 4],
                    },
                    Vertex {
                        position: Vec3::from((offset, -0.5)),
                        uv: uv_pos + uv_rad * offset,
                        normal: Vec4::new(0.0, 0.0, -1.0, 1.0),
                        color: [255; 4],
                    },
                ]
            }))
            .collect(),
        indices: (0..n_vertices)
            .flat_map(|i| {
                [0].into_iter()
                    .chain(
                        [0, 2, 0, 2, 1, 2, 3, 1, 3, 1]
                            .map(|j| (((2 * i + j) % (2 * n_vertices)) + 2) as u16),
                    )
                    .chain([1])
            })
            .collect(),
        texture: Some(texture),
    }
}

pub struct VehicleModel {
    config: VehicleConfig,

    wheel_model: Rc<Mesh>,
    model: Rc<Mesh>,
}

impl VehicleModel {
    pub fn new(config: VehicleConfig, mut model: Mesh, texture: Texture2D) -> Self {
        model.texture = Some(texture.clone());
        let wheel_model = Rc::new(make_wheel_model(
            32,
            config.wheel_common.texture_center,
            config.wheel_common.texture_radius,
            texture,
        ));
        Self {
            config: config.clone(),

            wheel_model,
            model: Rc::new(model),
        }
    }
}

pub struct Vehicle {
    config: VehicleConfig,

    pos: Var<Vec3>,
    rot: Var<Rot3>,

    vel: Var<Vec3>,
    /// Angular speed in rotating reference frame around principal axes of inertia coordinates
    rasp: Var<Vec3>,

    wheels: [Wheel; 4],

    model: Rc<Mesh>,
}

pub struct Wheel {
    common: WheelConfig,
    config: WheelInstanceConfig,

    axis: Vec3,
    rot: Var<Rot2>,
    /// Fixed angular speed
    fixed_asp: Option<f32>,
    /// Visible angular speed
    visible_asp: f32,

    dev: f32,

    model: Rc<Mesh>,
}

impl Wheel {
    fn new(common: WheelConfig, config: WheelInstanceConfig, model: Rc<Mesh>) -> Self {
        Self {
            common,
            config,
            axis: Vec3::X,
            rot: Var::default(),
            fixed_asp: None,
            visible_asp: 0.0,
            dev: 0.0,
            model,
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

    fn add_vel_at_poc(&self, vel: Vec3, normal: Vec3) -> Vec3 {
        if let Some(asp) = self.fixed_asp {
            angular_to_linear3(asp * self.axis, -self.common.radius * Vec3::Z)
        } else {
            -vel.project_onto(self.axis.cross(normal))
        }
    }

    fn set_visible_asp(&mut self, vel: Vec3) {
        if let Some(asp) = self.fixed_asp {
            self.visible_asp = asp;
        } else {
            let r = -self.common.radius * Vec3::Z;
            self.visible_asp = vel.cross(r).dot(self.axis) / self.common.radius.powi(2);
        }
    }

    /// Returns force applied
    fn normal_reaction(&mut self, normal: Vec3, vel_at: Vec3) -> Vec3 {
        let susp = self.dev * (self.common.hardness.0 + self.dev * self.common.hardness.1)
            - vel_at.dot(Vec3::Z) * self.common.damping;

        (susp * Vec3::Z).project_onto_normalized(normal)
    }

    /// Returns force applied
    fn friction(
        &mut self,
        normal_reaction: Vec3,
        dry_friction: bool,
        vel_at: Vec3,
        acc_at: Vec3,
        eff_mass: f32,
        dt: f32,
    ) -> Vec3 {
        let stiction = -eff_mass * (vel_at / dt + acc_at).reject_from(normal_reaction);

        let force_abs = stiction.length();
        let force_abs_max = Terrain::DRY_FRICTION * normal_reaction.length();

        if force_abs < force_abs_max {
            stiction
        } else if dry_friction {
            stiction * (force_abs_max / force_abs)
        } else {
            Vec3::ZERO
        }
    }

    fn draw(&self, stack: &mut impl TransformStack) {
        let _t = stack.push(Affine3A::from_scale_rotation_translation(
            Vec3::new(self.common.radius, self.common.radius, self.common.width),
            Quat::from_rotation_z(self.axis.y.atan2(self.axis.x))
                * Quat::from_rotation_y(0.5 * PI)
                * Quat::from_rotation_z(self.rot.angle()),
            self.center(),
        ));
        draw_mesh(&self.model);
    }
}

impl Vehicle {
    pub fn new(model: &VehicleModel, pos: Vec3, rot: Quat) -> Self {
        Self {
            config: model.config.clone(),

            pos: Var::new(pos),
            rot: Var::new(Rot3::from(rot)),

            vel: Var::default(),
            rasp: Var::default(),

            wheels: model.config.wheels.clone().map(|wc| {
                Wheel::new(
                    model.config.wheel_common.clone(),
                    wc,
                    model.wheel_model.clone(),
                )
            }),

            model: model.model.clone(),
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
    pub fn accelerate(&mut self, throttle: f32) {
        // All wheels
        for wheel in &mut self.wheels {
            wheel.fixed_asp = Some(-throttle * 10.0);
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

    pub fn compute_basic_derivs(&mut self) {
        self.pos.add_deriv(*self.vel);
        self.rot.add_deriv(self.rot.transform(*self.rasp));

        self.vel.add_deriv(GRAVITY);

        let inert = self.config.principal_moments_of_inertia;
        // According to Euler's equation
        self.rasp
            .add_deriv(-self.rasp.cross(inert * *self.rasp) / inert);

        for wheel in &mut self.wheels {
            wheel.rot.add_deriv(wheel.visible_asp);
        }
    }

    pub fn interact_with_terrain(&mut self, terrain: &Terrain, dt: f32) {
        let map = Affine3A::from_rotation_translation(Quat::from(*self.rot), *self.pos);
        let irot = self.rot.inverse();

        let mut normal_reactions = [None::<Vec3>; 4];
        for (wheel, normal_reaction) in self.wheels.iter_mut().zip(normal_reactions.iter_mut()) {
            if let Some((_poc, normal)) = wheel.contact_terrain(map, terrain) {
                // Use only local coordinates
                let normal = irot.transform(normal);
                let poc = wheel.poc();

                let mut vel_at = irot.transform(*self.vel) + angular_to_linear3(*self.rasp, poc);
                wheel.set_visible_asp(vel_at);
                vel_at += wheel.add_vel_at_poc(vel_at, normal);

                let force = wheel.normal_reaction(normal, vel_at);
                *normal_reaction = Some(force);

                self.vel
                    .add_deriv(self.rot.transform(force) / self.config.mass);
                self.rasp
                    .add_deriv(torque3(poc, force) / self.config.principal_moments_of_inertia);
            }
        }

        for i in 0..2 {
            for (wheel, normal_reaction) in self.wheels.iter_mut().zip(normal_reactions) {
                if let Some(normal_reaction) = normal_reaction {
                    // Use only local coordinates
                    let normal = normal_reaction.normalize();
                    let poc = wheel.poc();

                    let mut vel_at =
                        irot.transform(*self.vel) + angular_to_linear3(*self.rasp, poc);
                    vel_at += wheel.add_vel_at_poc(vel_at, normal);
                    let acc_at = irot.transform(*self.vel.deriv())
                        + angular_to_linear3(*self.rasp.deriv(), poc);

                    let dir = vel_at.reject_from_normalized(normal).normalize_or_zero();
                    let eff_mass = 1.0
                        / (1.0 / self.config.mass
                            + (dir.cross(poc))
                                .dot(dir.cross(poc) / self.config.principal_moments_of_inertia));

                    let force =
                        wheel.friction(normal_reaction, i == 0, vel_at, acc_at, eff_mass, dt);

                    self.vel
                        .add_deriv(self.rot.transform(force) / self.config.mass);
                    self.rasp
                        .add_deriv(torque3(poc, force) / self.config.principal_moments_of_inertia);
                }
            }
        }
    }

    pub fn visit_vars<V: Visitor>(&mut self, visitor: &mut V) {
        visitor.apply(&mut self.pos);
        visitor.apply(&mut self.rot);
        visitor.apply(&mut self.vel);
        visitor.apply(&mut self.rasp);
        for wheel in &mut self.wheels {
            visitor.apply(&mut wheel.rot);
        }
    }

    pub fn draw(&self, stack: &mut impl TransformStack) {
        let mut local = stack.push(Affine3A::from_rotation_translation(
            Quat::from(*self.rot),
            *self.pos,
        ));

        draw_mesh(&self.model);

        for wheel in &self.wheels {
            wheel.draw(&mut local);
        }
    }
}
