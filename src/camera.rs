use drive::terrain::Terrain;
use glam::{Mat4, Quat, Vec2, Vec3};
use std::f32::consts::{FRAC_PI_2, FRAC_PI_4, TAU};

pub struct Orbit {
    distance: f32,
    azimuth: f32,
    elevation: f32,
}
impl Default for Orbit {
    fn default() -> Self {
        Self {
            distance: 10.0,
            azimuth: 0.0,
            elevation: -FRAC_PI_4,
        }
    }
}
impl Orbit {
    pub fn rotate(&mut self, delta: Vec2) {
        self.azimuth = (self.azimuth - 0.002 * delta.x).rem_euclid(TAU);
        self.elevation =
            (self.elevation - 0.002 * delta.y).clamp(-FRAC_PI_2 + 0.02, FRAC_PI_2 - 0.02);
    }
    pub fn zoom(&mut self, scroll: f32) {
        self.distance = (self.distance * (-0.2 * scroll.clamp(-1.0, 1.0)).exp()).clamp(1.0, 100.0);
    }
    pub fn view(&self, target: Vec3, terrain: &Terrain, aspect: f32) -> Mat4 {
        let rotation = Quat::from_rotation_z(self.azimuth) * Quat::from_rotation_x(self.elevation);
        let wanted = target + rotation * Vec3::new(0.0, -self.distance, 0.0);
        // Trace out from the target so the first surface blocks the camera even
        // when both endpoints are above terrain. Keep a near-plane clearance.
        let position = if let Some((distance, _, _)) = terrain.intersect_line(target, wanted) {
            target + (wanted - target).normalize() * (distance - 0.15).max(0.2)
        } else {
            wanted
        };
        glam::camera::rh::proj::directx::perspective(1.0, aspect, 0.05, 1000.0)
            * glam::camera::rh::view::look_at_mat4(position, target, rotation * Vec3::Z)
    }
}
