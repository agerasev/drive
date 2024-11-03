use glam::{Vec3, Vec4};
use macroquad::{color::Color, texture::Texture2D};
use rand::Rng;
use rand_distr::Uniform;

pub fn noisy_texture<R: Rng>(rng: R, width: u16, height: u16, base: Vec3, var: Vec3) -> Texture2D {
    Texture2D::from_rgba8(
        width,
        height,
        &rng.sample_iter(Uniform::new(0.0, 1.0))
            .take(width as usize * height as usize)
            .flat_map(|a| Into::<[u8; 4]>::into(Color::from_vec(Vec4::from((base + a * var, 1.0)))))
            .collect::<Vec<u8>>(),
    )
}
