use crate::model::TransformStack;
use macroquad::{
    math::{Affine3A, Mat3, Quat, Vec2, Vec3, Vec4},
    models::{draw_mesh, Mesh},
    texture::Texture2D,
    ui::Vertex,
};
use serde::{Deserialize, Deserializer};
use std::f32::consts::PI;

fn de_vec2<'de, D: Deserializer<'de>>(deserializer: D) -> Result<Vec2, D::Error> {
    Ok(Vec2::from(<[f32; 2]>::deserialize(deserializer)?))
}

fn de_vec3<'de, D: Deserializer<'de>>(deserializer: D) -> Result<Vec3, D::Error> {
    Ok(Vec3::from(<[f32; 3]>::deserialize(deserializer)?))
}

fn de_mat3<'de, D: Deserializer<'de>>(deserializer: D) -> Result<Mat3, D::Error> {
    Ok(Mat3::from_cols_array_2d(&<[[f32; 3]; 3]>::deserialize(
        deserializer,
    )?))
}

#[derive(Clone, Debug, Deserialize)]
pub struct VehicleConfig {
    /// Mass (kg)
    pub mass: f32,
    /// Moment of inertia tensor (kg*m^2)
    #[serde(deserialize_with = "de_mat3")]
    pub moment_of_inertia: Mat3,

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

    #[serde(deserialize_with = "de_vec2")]
    pub texture_center: Vec2,
    pub texture_radius: f32,
}

#[derive(Clone, Debug, Deserialize)]
pub struct WheelInstanceConfig {
    #[serde(deserialize_with = "de_vec3")]
    pub position: Vec3,
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

    model: Mesh,
    wheel_model: Mesh,
}

impl Vehicle {
    pub fn new(config: VehicleConfig, mut model: Mesh, texture: Texture2D) -> Self {
        model.texture = Some(texture.clone());
        let wheel_model = make_wheel_model(
            64,
            config.wheel_common.texture_center,
            config.wheel_common.texture_radius,
            texture,
        );
        Self {
            config,
            model,
            wheel_model,
        }
    }

    pub fn draw(&self, stack: &TransformStack) {
        draw_mesh(&self.model);

        let wheel_common = &self.config.wheel_common;
        for wheel in &self.config.wheels {
            let _transform = stack.push(Affine3A::from_scale_rotation_translation(
                Vec3::new(wheel_common.radius, wheel_common.radius, wheel_common.width),
                Quat::from_rotation_y(0.5 * PI),
                wheel.position,
            ));
            draw_mesh(&self.wheel_model);
        }
    }
}
