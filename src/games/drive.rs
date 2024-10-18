use crate::model::load_model;
use anyhow::Error;
use defer::defer;
use macroquad::{
    camera::{set_camera, set_default_camera, Camera, Camera3D},
    color,
    file::load_file,
    input::{
        is_key_down, is_key_pressed, is_mouse_button_down, mouse_delta_position, mouse_wheel,
        set_cursor_grab, show_mouse, KeyCode, MouseButton,
    },
    math::{Affine3A, EulerRot, Mat3, Mat4, Quat, Rect, Vec2, Vec3, Vec4},
    miniquad::window::screen_size,
    models::{draw_mesh, Mesh},
    texture::{load_texture, set_default_filter_mode, FilterMode, RenderPass, Texture2D},
    ui::Vertex,
    window::{clear_background, next_frame},
};
use serde::{Deserialize, Deserializer};
use std::{f32::consts::PI, future::Future, pin::Pin};

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
struct VehicleConfig {
    /// Mass (kg)
    mass: f32,
    /// Moment of inertia tensor (kg*m^2)
    #[serde(deserialize_with = "de_mat3")]
    moment_of_inertia: Mat3,

    wheel_common: WheelConfig,
    wheels: [WheelInstanceConfig; 4],
}

#[derive(Clone, Debug, Deserialize)]
struct WheelConfig {
    radius: f32,
    width: f32,
    /// Mass of wheel (kg)
    mass: f32,
    /// Moment of inertia around wheel axis (kg*m^2)
    moment_of_inertia: f32,

    /// Spring linear and quadratic hardness (N/m, N/m^2)
    hardness: (f32, f32),
    /// Liquid friction of shock absorber (N/(m/s))
    damping: f32,

    #[serde(deserialize_with = "de_vec2")]
    texture_center: Vec2,
    texture_radius: f32,
}

#[derive(Clone, Debug, Deserialize)]
struct WheelInstanceConfig {
    #[serde(deserialize_with = "de_vec3")]
    position: Vec3,
}

fn make_wheel_model(n_vertices: usize, tex_pos: Vec2, tex_rad: f32, texture: Texture2D) -> Mesh {
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

struct Vehicle {
    config: VehicleConfig,

    model: Mesh,
    wheel_model: Mesh,
}

impl Vehicle {
    fn new(config: VehicleConfig, mut model: Mesh, texture: Texture2D) -> Self {
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

    fn draw(&self, stack: &TransformStack) {
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

enum TransformStack<'a> {
    Camera(&'a Camera3D),
    Transform(&'a Self, Mat4),
}

impl<'a> Camera for TransformStack<'a> {
    fn matrix(&self) -> Mat4 {
        match self {
            Self::Camera(camera) => camera.matrix(),
            Self::Transform(base, transform) => base.matrix().mul_mat4(transform),
        }
    }
    fn depth_enabled(&self) -> bool {
        match self {
            Self::Camera(camera) => camera.depth_enabled(),
            Self::Transform(base, _) => base.depth_enabled(),
        }
    }
    fn render_pass(&self) -> Option<RenderPass> {
        match self {
            Self::Camera(camera) => camera.render_pass(),
            Self::Transform(base, _) => base.render_pass(),
        }
    }
    fn viewport(&self) -> Option<(i32, i32, i32, i32)> {
        match self {
            Self::Camera(camera) => camera.viewport(),
            Self::Transform(base, _) => base.viewport(),
        }
    }
}

impl<'a> TransformStack<'a> {
    fn new(camera: &'a Camera3D) -> Self {
        let this = Self::Camera(camera);
        set_camera(&this);
        this
    }
    fn push(&self, transform: Affine3A) -> TransformStack {
        let this = TransformStack::Transform(self, Mat4::from(transform));
        set_camera(&this);
        this
    }
}

impl<'a> Drop for TransformStack<'a> {
    fn drop(&mut self) {
        if let Self::Transform(base, _) = self {
            set_camera(*base);
        }
    }
}

fn grab(state: bool) {
    set_cursor_grab(state);
    show_mouse(!state);
}

pub async fn main() -> Result<(), Error> {
    set_default_filter_mode(FilterMode::Linear);
    let vehicle = Vehicle::new(
        serde_json::from_slice(&load_file("l200.json").await?)?,
        load_model("l200.obj").await?,
        load_texture("l200.png").await?,
    );

    let mut grabbed = true;
    grab(grabbed);
    defer!(grab(false));

    let mouse_sens = Vec2::new(5e-4, 5e-4);
    let wheel_sens: f32 = 0.2;
    let (mut r, mut phi, mut theta) = (10.0, 0.0, 0.0);
    while !is_key_down(KeyCode::Escape) {
        {
            if is_key_pressed(KeyCode::Tab) {
                grabbed ^= true;
                grab(grabbed);
            }
            let scroll = mouse_wheel().1;
            if scroll != 0.0 {
                r *= (1.0 + wheel_sens).powf(-mouse_wheel().1);
            } else if grabbed || is_mouse_button_down(MouseButton::Left) {
                let delta = mouse_sens * mouse_delta_position() * Vec2::from(screen_size());
                phi = (phi + delta.x) % (2.0 * PI);
                theta = (theta + delta.y).clamp(-0.5 * PI, 0.5 * PI);
            }
        }

        {
            let transorm = Quat::from_euler(EulerRot::ZXY, phi, theta, 0.0);
            let camera = Camera3D {
                target: Vec3::ZERO,
                position: transorm.mul_vec3(Vec3::new(0.0, -r, 0.0)),
                up: transorm.mul_vec3(Vec3::new(0.0, 0.0, 1.0)),
                ..Default::default()
            };
            let stack = TransformStack::new(&camera);

            clear_background(color::BLACK);

            vehicle.draw(&stack);

            set_default_camera();
        }

        next_frame().await
    }

    Ok(())
}

pub struct Game {}

impl Game {
    pub async fn new() -> Result<Self, Error> {
        Ok(Self {})
    }
}

impl crate::Game for Game {
    fn name(&self) -> String {
        "Машина".to_owned()
    }

    fn draw_preview(&self, _rect: Rect) {}

    fn launch(&self) -> Pin<Box<dyn Future<Output = Result<(), Error>>>> {
        Box::pin(main())
    }
}
