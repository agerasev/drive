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

const GRAVITY: Vec3 = Vec3::new(0.0, 0.0, -4.0);

/// How fast dry frinction grow depending on speed.
///
/// Ideally this should be infinity, but then we cannot solve it by RK4.  
const DRY_FRICTION_SLOPE: f32 = 1.0;

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

pub struct Vehicle {
    config: VehicleConfig,

    pos: Var<Vec3>,
    rot: Var<Rot3>,

    vel: Var<Vec3>,
    /// Angular speed in rotating reference frame around principal axes of inertia coordinates
    rasp: Var<Vec3>,

    wheels: [Wheel; 4],

    model: Mesh,
}

pub struct Wheel {
    common: WheelConfig,
    config: WheelInstanceConfig,

    axis: Vec3,
    rot: Var<Rot2>,
    /// Angular speed
    asp: Var<f32>,

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
            asp: Var::default(),
            dev: 0.0,
            model,
        }
    }

    pub fn pos(&self) -> Vec3 {
        self.config.center + self.dev * Vec3::Z
    }
    pub fn lower_poc(&self) -> Vec3 {
        self.config.center - self.common.radius * Vec3::Z
    }
    pub fn upper_poc(&self) -> Vec3 {
        self.config.center + (self.common.travel - self.common.radius) * Vec3::Z
    }

    /// Returns point of contact and force applied
    fn interact_with_terrain(
        &mut self,
        map: Affine3A,
        vel_at: Vec3,
        terrain: &Terrain,
    ) -> Option<(Vec3, Vec3)> {
        if let Some((dist, poc, normal)) = terrain.intersect_line(
            map.transform_point3(self.upper_poc()),
            map.transform_point3(self.lower_poc()),
        ) {
            self.dev = self.common.travel - dist;

            let susp = self.dev * (self.common.hardness.0 + self.dev * self.common.hardness.1)
                - vel_at.dot(map.transform_vector3(Vec3::Z)) * self.common.damping;
            let normal_reaction = map
                .transform_vector3(susp * Vec3::Z)
                .project_onto_normalized(normal);

            let rel_poc = poc - map.transform_point3(self.pos());
            let vel_at =
                vel_at + angular_to_linear3(map.transform_vector3(*self.asp * self.axis), rel_poc);
            let friction = (-vel_at * DRY_FRICTION_SLOPE)
                .reject_from_normalized(normal)
                .clamp_length_max(Terrain::DRY_FRICTION)
                * normal_reaction.length();

            self.asp.add_deriv(
                torque3(rel_poc, friction).dot(map.transform_vector3(self.axis))
                    / self.common.moment_of_inertia,
            );

            Some((poc, normal_reaction + friction))
        } else {
            self.dev = 0.0;
            None
        }
    }

    fn draw(&self, stack: &mut impl TransformStack) {
        let _t = stack.push(Affine3A::from_scale_rotation_translation(
            Vec3::new(self.common.radius, self.common.radius, self.common.width),
            Quat::from_rotation_y(0.5 * PI) * Quat::from_rotation_z(self.rot.angle()),
            self.pos(),
        ));
        draw_mesh(&self.model);
    }
}

impl Vehicle {
    pub fn new(
        config: VehicleConfig,
        pos: Vec3,
        rot: Quat,
        mut model: Mesh,
        texture: Texture2D,
    ) -> Self {
        model.texture = Some(texture.clone());
        let wheel_model = Rc::new(make_wheel_model(
            64,
            config.wheel_common.texture_center,
            config.wheel_common.texture_radius,
            texture,
        ));
        Self {
            config: config.clone(),

            pos: Var::new(pos),
            rot: Var::new(Rot3::from(rot)),

            vel: Var::default(),
            rasp: Var::default(),

            wheels: config
                .wheels
                .map(|wc| Wheel::new(config.wheel_common.clone(), wc, wheel_model.clone())),

            model,
        }
    }

    pub fn pos(&self) -> Vec3 {
        *self.pos
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
            wheel.rot.add_deriv(*wheel.asp);
        }
    }

    pub fn interact_with_terrain(&mut self, terrain: &Terrain) {
        let map = Affine3A::from_rotation_translation(Quat::from(*self.rot), *self.pos);

        for wheel in self.wheels.iter_mut() {
            let vel_at =
                *self.vel + (self.rot).transform(angular_to_linear3(*self.rasp, wheel.upper_poc()));
            if let Some((poc, force)) = wheel.interact_with_terrain(map, vel_at, terrain) {
                self.vel.add_deriv(force / self.config.mass);
                self.rasp.add_deriv(self.rot.inverse().transform(
                    torque3(poc - *self.pos, force) / self.config.principal_moments_of_inertia,
                ));
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
            visitor.apply(&mut wheel.asp);
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
