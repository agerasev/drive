mod pipeline;
mod terrain;
mod vehicle;

use self::vehicle::Vehicle;
use crate::{
    compat::mouse_wheel_clamped,
    model::load_model,
    numerical::{Solver, System, Visitor},
    texture::noisy_texture,
};
use anyhow::Error;
use defer::defer;
use glam::{EulerRot, Quat, Vec2, Vec3};
use macroquad::{
    camera::{set_camera, set_default_camera, Camera, Camera3D},
    color,
    file::load_file,
    input::{
        is_key_down, is_key_pressed, is_mouse_button_down, mouse_delta_position,
        mouse_position_local, set_cursor_grab, show_mouse, KeyCode, MouseButton,
    },
    math::Rect,
    miniquad::window::screen_size,
    models::draw_sphere,
    texture::{load_texture, set_default_filter_mode, FilterMode},
    time::{get_frame_time, get_time},
    window::{clear_background, next_frame, screen_height},
};
use pipeline::Pipeline;
use rand::{rngs::SmallRng, SeedableRng};
use std::{f32::consts::PI, future::Future, pin::Pin, rc::Rc};
use terrain::Terrain;
use vehicle::{load_wheel, VehicleModel};

impl System for (&Terrain, &mut Vehicle) {
    fn compute_derivs(&mut self, dt: f32) {
        self.1.compute_basic_derivs();
        self.1.interact_with_terrain(self.0, dt);
    }

    fn visit_vars<V: Visitor>(&mut self, visitor: &mut V) {
        self.1.visit_vars(visitor);
    }
}

pub async fn main() -> Result<(), Error> {
    let mut rng = SmallRng::seed_from_u64(0xdeadbeef);
    set_default_filter_mode(FilterMode::Linear);

    let terrain = Terrain::from_height_map(
        |c| 8.0 * (1.0 - 1.0 / (1.0 + 0.002 * c.length_squared())),
        64.0,
        24,
        noisy_texture(
            &mut rng,
            256,
            256,
            Vec3::new(0.0, 0.50, 0.0),
            Vec3::new(0.25, 0.25, 0.25),
        ),
    );
    let wheel = Rc::new(load_wheel().await?);
    let model = VehicleModel::new(
        serde_json::from_slice(&load_file("logan/config.json").await?)?,
        load_model("logan/model.obj").await?,
        load_texture("logan/color.png").await?,
        wheel,
    );

    let pipeline = Pipeline::new()?;

    fn grab(state: bool) {
        show_mouse(!state);
        set_cursor_grab(state);
    }
    let mut grabbed = true;
    grab(grabbed);
    defer!(grab(false));

    let mouse_sens = Vec2::new(5e-4, 5e-4);
    let wheel_sens: f32 = 0.2;

    'outer: loop {
        let mut vehicle = Vehicle::new(&model, Vec3::new(4.0, 4.0, 3.0), Quat::IDENTITY);

        let (mut r, mut phi, mut theta) = (10.0, 0.0, -PI / 4.0);
        loop {
            let dt = get_frame_time().max(0.04) * 0.5;

            {
                if is_key_down(KeyCode::Escape) {
                    break 'outer;
                }

                if is_key_pressed(KeyCode::Tab) {
                    grabbed ^= true;
                    grab(grabbed);
                }
                let scroll = mouse_wheel_clamped().1;
                if scroll != 0.0 {
                    r *= (1.0 + wheel_sens).powf(-scroll);
                } else if grabbed || is_mouse_button_down(MouseButton::Left) {
                    // On some platforms scroll affects mouse delta
                    let delta = mouse_sens * mouse_delta_position() * Vec2::from(screen_size());
                    phi = (phi + delta.x) % (2.0 * PI);
                    theta = (theta + delta.y).clamp(-0.5 * PI, 0.5 * PI);
                }

                vehicle.reset_controls();
                let fwd = is_key_down(KeyCode::W) || is_key_down(KeyCode::Up);
                let bwd = is_key_down(KeyCode::S) || is_key_down(KeyCode::Down);
                let brake = is_key_down(KeyCode::Space) || (fwd && bwd);
                if brake {
                    vehicle.brake();
                }
                if !brake {
                    if fwd {
                        vehicle.accelerate(1.0);
                    } else if bwd {
                        vehicle.accelerate(-0.5);
                    }
                }
                let dir = (is_key_down(KeyCode::A) || is_key_down(KeyCode::Left)) as i32
                    - (is_key_down(KeyCode::D) || is_key_down(KeyCode::Right)) as i32;
                vehicle.steer((PI / 6.0) * dir as f32);
            }

            {
                Solver.solve_step(&mut (&terrain, &mut vehicle), dt);

                if vehicle.pos().z < -100.0 {
                    break;
                }
            }

            {
                let transform = Quat::from_euler(EulerRot::ZXY, phi, theta, 0.0);
                let mut camera = Camera3D {
                    target: vehicle.pos(),
                    position: vehicle.pos() + transform.mul_vec3(Vec3::new(0.0, -r, 0.0)),
                    up: transform.mul_vec3(Vec3::new(0.0, 0.0, 1.0)),
                    ..Default::default()
                };
                if let Some((_, poc, _)) = terrain.intersect_line(camera.position, camera.target) {
                    camera.position = poc;
                }
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
                vehicle.draw(&mut camera, Some(&pipeline));

                set_default_camera();
            }

            next_frame().await
        }
    }

    Ok(())
}

pub struct Game {
    vehicle: VehicleModel,
}

impl Game {
    pub async fn new() -> Result<Self, Error> {
        set_default_filter_mode(FilterMode::Linear);
        Ok(Self {
            vehicle: VehicleModel::new(
                serde_json::from_slice(&load_file("l200/config.json").await?)?,
                load_model("l200/model.obj").await?,
                load_texture("l200/color.png").await?,
                Rc::new(load_wheel().await?),
            ),
        })
    }
}

impl crate::Game for Game {
    fn name(&self) -> String {
        "Машина".to_owned()
    }

    fn draw_preview(&self, rect: Rect) {
        let vehicle = Vehicle::new(&self.vehicle, Vec3::new(4.0, 4.0, 3.0), Quat::IDENTITY);
        let transform = Quat::from_euler(EulerRot::ZXY, PI - 0.1 * get_time() as f32, -0.5, 0.0);
        let mut camera = Camera3D {
            target: vehicle.pos(),
            position: vehicle.pos() + transform.mul_vec3(Vec3::new(0.0, -8.0, 0.0)),
            up: transform.mul_vec3(Vec3::new(0.0, 0.0, 1.0)),
            viewport: Some((
                rect.x as i32,
                (screen_height() - rect.bottom()) as i32,
                rect.w as i32,
                rect.h as i32,
            )),
            aspect: Some(rect.w / rect.h),
            ..Default::default()
        };
        set_camera(&camera);
        vehicle.draw(&mut camera, None)
    }

    fn launch(&self) -> Pin<Box<dyn Future<Output = Result<(), Error>>>> {
        Box::pin(main())
    }
}
