mod camera;
mod render;
mod timing;
use camera::Orbit;
use drive::{config::VehicleConfig, terrain::Terrain, vehicle::Vehicle};
use glam::{Quat, Vec3};
use wgame::{
    Library, Result, Window,
    app::time::Instant,
    canvas::{Button, Event, Key},
    gfx::{Camera, Scene},
    prelude::*,
};

fn config(model: usize) -> Result<VehicleConfig> {
    Ok(serde_json::from_slice(if model == 0 {
        &include_bytes!("../assets/logan/config.json")[..]
    } else {
        &include_bytes!("../assets/l200/config.json")[..]
    })?)
}
fn spawn(model: usize) -> Result<Vehicle> {
    Ok(Vehicle::new(
        config(model)?,
        Vec3::new(4.0, 4.0, 3.0),
        Quat::IDENTITY,
    ))
}
fn grab(window: &wgame::app::RawWindow, enable: bool) -> bool {
    // winit is already exposed through wgame's window; its cursor modes are
    // imported directly below because they are platform policy, not scene state.
    let result = if enable {
        window
            .set_cursor_grab(winit::window::CursorGrabMode::Locked)
            .or_else(|_| window.set_cursor_grab(winit::window::CursorGrabMode::Confined))
    } else {
        window.set_cursor_grab(winit::window::CursorGrabMode::None)
    };
    let captured = enable && result.is_ok();
    window.set_cursor_visible(!captured);
    captured
}
struct Capture<'a>(&'a wgame::app::RawWindow);
impl Drop for Capture<'_> {
    fn drop(&mut self) {
        grab(self.0, false);
    }
}

#[wgame::window(title = "Drive — WASD, Space: brake, mouse: orbit, Tab: capture", logical_size = (1280.0,720.0), resizable = true, vsync = true)]
async fn main(mut window: Window<'_>) -> Result<()> {
    let lib = Library::new(window.graphics());
    let terrain = Terrain::from_height_map(
        |c| 8.0 * (1.0 - 1.0 / (1.0 + 0.002 * c.length_squared())),
        64.0,
        24,
    );
    let assets = render::Assets::new(&lib, &terrain)?;
    let mut model = 0;
    let mut car = spawn(model)?;
    let mut orbit = Orbit::default();
    let raw = window.raw();
    let _capture = Capture(raw);
    #[cfg(not(target_arch = "wasm32"))]
    let smoke = std::env::args().any(|arg| arg == "--smoke");
    #[cfg(target_arch = "wasm32")]
    let smoke = false;
    let mut captured = if cfg!(target_arch = "wasm32") || smoke {
        false
    } else {
        grab(raw, true)
    };
    let mut paused = false;
    let mut slow = false;
    let mut pointer = None;
    let mut clock = timing::Clock::default();
    let mut last = Instant::now();
    let mut frames = 0;
    'frames: while let Some(mut frame) = window.next_frame().await? {
        let now = Instant::now();
        let elapsed = now - last;
        last = now;
        let mut reset = false;
        let mut scroll = 0.0;
        for event in &frame.input().events {
            match *event {
                Event::Key {
                    key,
                    pressed: true,
                    repeat: false,
                } => match key {
                    Key::Escape => {
                        frame.discard();
                        break 'frames;
                    }
                    Key::Tab => {
                        captured = grab(raw, !captured);
                        pointer = None;
                    }
                    Key::Character('p') => {
                        paused = !paused;
                        reset = true;
                    }
                    Key::Character('t') => {
                        slow = !slow;
                        reset = true;
                    }
                    Key::Character('r') => {
                        car = spawn(model)?;
                        reset = true;
                    }
                    Key::Character('1' | '2') => {
                        model = usize::from(key == Key::Character('2'));
                        car = spawn(model)?;
                        reset = true;
                    }
                    _ => {}
                },
                Event::Scroll(delta) => scroll += delta.y / 40.0,
                Event::Moved(pos) => {
                    if !captured
                        && frame.input().button_down(Button::Primary)
                        && let Some(previous) = pointer
                    {
                        orbit.rotate(pos - previous);
                    }
                    pointer = Some(pos);
                }
                Event::Cancelled | Event::Focused(false) => {
                    reset = true;
                    pointer = None;
                    if !frame.input().window_focused {
                        captured = grab(raw, false);
                    }
                }
                _ => {}
            }
        }
        if scroll != 0.0 {
            orbit.zoom(scroll);
        } else if captured {
            orbit.rotate(frame.input().relative_motion);
        }
        let held = |arrow, letter| {
            frame.input().key_down(arrow) || frame.input().key_down(Key::Character(letter))
        };
        let forward = held(Key::ArrowUp, 'w');
        let back = held(Key::ArrowDown, 's');
        car.reset_controls();
        if frame.input().key_down(Key::Space) || (forward && back) {
            car.brake();
        } else if forward {
            car.accelerate(1.0);
        } else if back {
            car.accelerate(-0.5);
        }
        let steering = i32::from(held(Key::ArrowLeft, 'a')) - i32::from(held(Key::ArrowRight, 'd'));
        car.steer(std::f32::consts::FRAC_PI_6 * steering as f32);
        for _ in 0..clock.steps(
            if smoke { timing::STEP * 4 } else { elapsed },
            !paused && !reset && frame.input().window_focused,
            slow,
        ) {
            car.step(&terrain, timing::STEP.as_secs_f32());
        }
        if car.pos().z < -100.0 {
            car = spawn(model)?;
        }
        let (width, height) = frame.size();
        let camera = Camera::new(
            lib.state(),
            orbit.view(car.pos(), &terrain, width as f32 / height as f32),
        );
        frame.clear(wgame::rgb::Rgb::new(0.5_f32, 0.5, 0.5));
        frame.render(&camera, &assets.terrain);
        let mut scene = Scene::default();
        assets.draw_vehicle(&car, model, &mut scene);
        if !captured
            && let Some(pos) = frame.input().pointer
            && let Some((origin, dir)) =
                camera.screen_ray(pos * frame.scale_factor() as f32, frame.size())
            && let Some((dist, hit, _)) = terrain.intersect_line(origin, origin + dir * 1000.0)
        {
            assets.draw_marker(hit, (0.01 * dist).clamp(0.03, 1.0), &mut scene);
        }
        frame.render_iter(&camera, scene.iter());
        frame.present();
        frames += 1;
        if smoke && frames >= 12 {
            break;
        }
    }
    Ok(())
}
