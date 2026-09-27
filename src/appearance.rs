//! Per-vehicle paint choices; picker values are sRGB, shader colors are linear.
use glam::Vec3;

pub const VEHICLE_NAMES: [&str; 2] = ["Renault Logan", "Mitsubishi L200"];
pub const DEFAULT_PAINT: [[u8; 3]; 2] = [[103, 107, 103], [183, 186, 205]];

#[derive(Clone, Copy, Debug)]
pub struct Appearance {
    pub model: usize,
    pub colors: [[u8; 3]; 2],
}
impl Default for Appearance {
    fn default() -> Self {
        Self {
            model: 0,
            colors: DEFAULT_PAINT,
        }
    }
}
pub fn linear_color(srgb: [u8; 3]) -> Vec3 {
    Vec3::from_array(srgb.map(|c| {
        let c = f32::from(c) / 255.0;
        if c <= 0.04045 {
            c / 12.92
        } else {
            ((c + 0.055) / 1.055).powf(2.4)
        }
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn picker_colors_are_converted_from_srgb_once() {
        assert_eq!(linear_color([0; 3]), Vec3::ZERO);
        assert_eq!(linear_color([255; 3]), Vec3::ONE);
        let mid = linear_color([128, 0, 255]);
        assert!((mid.x - 0.2158605).abs() < 1e-6);
        assert_eq!(mid.y, 0.0);
        assert_eq!(mid.z, 1.0);
    }
}
