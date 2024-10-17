use crate::model::load_model;
use anyhow::Error;
use defer::defer;
use macroquad::{
    camera::{set_camera, set_default_camera, Camera3D},
    color,
    file::load_file,
    input::{
        is_key_down, is_key_pressed, is_mouse_button_down, mouse_delta_position, mouse_wheel,
        set_cursor_grab, show_mouse, KeyCode, MouseButton,
    },
    math::{EulerRot, Quat, Rect, Vec2, Vec3},
    miniquad::window::screen_size,
    models::{draw_mesh, Mesh},
    texture::{load_texture, set_default_filter_mode, FilterMode},
    window::{clear_background, next_frame},
};
use serde::{Deserialize, Deserializer};
use std::{f32::consts::PI, future::Future, pin::Pin};

fn de_vec3<'de, D: Deserializer<'de>>(deserializer: D) -> Result<Vec3, D::Error> {
    Ok(Vec3::from(<[f32; 3]>::deserialize(deserializer)?))
}

#[derive(Clone, Debug, Deserialize)]
struct WheelConfig {
    /// Position when no force applied
    #[serde(deserialize_with = "de_vec3")]
    position: Vec3,
    radius: f32,
    width: f32,
    /// Spring linear and quadratic hardness (N/m, N/m^2)
    hardness: (f32, f32),
    /// Suspension (N/(m/s))
    suspension: f32,
}

#[derive(Clone, Debug, Deserialize)]
struct VehicleConfig {
    /// Mass in kg
    mass: f32,

    /// Wheels in order:
    ///
    /// + front-left
    /// + front-right
    /// + rear-right
    /// + rear-left
    wheels: [WheelConfig; 4],
}

fn grab(state: bool) {
    set_cursor_grab(state);
    show_mouse(!state);
}

pub async fn main() -> Result<(), Error> {
    set_default_filter_mode(FilterMode::Linear);
    let model = Mesh {
        texture: Some(load_texture("l200.png").await?),
        ..load_model("l200.obj").await?
    };
    let config = serde_json::from_slice(&load_file("l200.json").await?)?;

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
            set_camera(&camera);

            clear_background(color::BLACK);

            draw_mesh(&model);

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
