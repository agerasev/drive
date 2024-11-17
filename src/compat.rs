use macroquad::{
    camera::{set_camera, set_default_camera, Camera2D},
    input::mouse_wheel,
};

pub fn reset_camera() {
    // Workaround to reset viewport
    set_camera(&Camera2D::default());

    set_default_camera();
}

pub fn mouse_wheel_clamped() -> (f32, f32) {
    let (x, y) = mouse_wheel();
    (x.clamp(-1.0, 1.0), y.clamp(-1.0, 1.0))
}
