mod terrain;
mod vehicle;

use self::vehicle::Vehicle;
use crate::{
    model::load_model,
    physics::{Solver, System, Visitor},
};
use anyhow::Error;
use defer::defer;
use macroquad::{
    camera::{set_camera, set_default_camera, Camera, Camera3D},
    color,
    file::load_file,
    input::{
        is_key_down, is_key_pressed, is_mouse_button_down, mouse_delta_position,
        mouse_position_local, mouse_wheel, set_cursor_grab, show_mouse, KeyCode, MouseButton,
    },
    math::{EulerRot, Quat, Rect, Vec2, Vec3},
    miniquad::window::screen_size,
    models::draw_sphere,
    texture::{load_texture, set_default_filter_mode, FilterMode},
    time::get_frame_time,
    window::{clear_background, next_frame},
};
use rand::{rngs::SmallRng, SeedableRng};
use std::{f32::consts::PI, future::Future, pin::Pin};
use terrain::{noisy_texture, Terrain};

impl System for (&Terrain, &mut Vehicle) {
    fn compute_derivs(&mut self) {
        self.1.compute_basic_derivs();
        self.1.interact_with_terrain(self.0);
    }

    fn visit_vars<V: Visitor>(&mut self, visitor: &mut V) {
        self.1.visit_vars(visitor);
    }
}

pub async fn main() -> Result<(), Error> {
    let mut rng = SmallRng::from_entropy();

    set_default_filter_mode(FilterMode::Linear);

    let terrain = Terrain::from_height_map(
        |c| 4.0 - 8.0 / (1.0 + 0.002 * c.length_squared()),
        64.0,
        16,
        noisy_texture(
            &mut rng,
            256,
            256,
            Vec3::new(0.0, 0.50, 0.0),
            Vec3::new(0.25, 0.25, 0.25),
        ),
    );
    let mut vehicle = Vehicle::new(
        serde_json::from_slice(&load_file("l200.json").await?)?,
        Vec3::new(4.0, 4.0, -1.0),
        Quat::IDENTITY,
        load_model("l200.obj").await?,
        load_texture("l200.png").await?,
    );

    fn grab(state: bool) {
        show_mouse(!state);
        set_cursor_grab(state);
    }
    let mut grabbed = true;
    grab(grabbed);
    defer!(grab(false));

    let mouse_sens = Vec2::new(5e-4, 5e-4);
    let wheel_sens: f32 = 0.2;
    let (mut r, mut phi, mut theta) = (10.0, 0.0, -PI / 4.0);
    while !is_key_down(KeyCode::Escape) {
        let dt = get_frame_time().max(0.04);

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
            Solver.solve_step(&mut (&terrain, &mut vehicle), dt);
        }

        {
            let transorm = Quat::from_euler(EulerRot::ZXY, phi, theta, 0.0);
            let mut camera = Camera3D {
                target: vehicle.pos(),
                position: vehicle.pos() + transorm.mul_vec3(Vec3::new(0.0, -r, 0.0)),
                up: transorm.mul_vec3(Vec3::new(0.0, 0.0, 1.0)),
                ..Default::default()
            };
            set_camera(&camera);

            clear_background(color::GRAY);

            if !grabbed {
                let origin = camera.position;
                let mouse_pos = camera.matrix().inverse().project_point3(Vec3::from((
                    mouse_position_local() * Vec2::new(1.0, -1.0),
                    0.98,
                )));
                let mouse_dir = mouse_pos - origin;
                if let Some((dist, poi, _)) =
                    terrain.intersect_line(origin, origin + 1e3 * mouse_dir)
                {
                    draw_sphere(poi, 0.01 * dist, None, color::RED);
                }
            }

            terrain.draw();
            vehicle.draw(&mut camera);

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
