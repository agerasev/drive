use macroquad::{
    color::Color,
    math::{Vec2, Vec3, Vec4},
    models::{draw_mesh, Mesh},
    texture::Texture2D,
    ui::Vertex,
};
use rand::Rng;
use rand_distr::Uniform;

use crate::geometry::Triangle3;

pub struct Terrain {
    mesh: Mesh,
    /// TODO: Use quad-tree
    tiles: Vec<Triangle3>,
}

#[derive(Clone, Copy, PartialEq, Debug)]
struct Point {
    pos: Vec3,
    normal: Vec3,
}

fn sample_height_map<F: Fn(Vec2) -> f32>(f: &F, coord: Vec2) -> Point {
    let pos = Vec3::from((coord, f(coord)));
    // Numerically compute normal
    let normal = {
        let delta = 1e-4;
        let px = Vec3::from((
            coord + Vec2::new(delta, 0.0),
            f(coord + Vec2::new(delta, 0.0)),
        )) - pos;
        let py = Vec3::from((
            coord + Vec2::new(0.0, delta),
            f(coord + Vec2::new(0.0, delta)),
        )) - pos;
        px.cross(py).try_normalize().unwrap()
    };
    Point { pos, normal }
}

impl Terrain {
    pub fn from_height_map<F: Fn(Vec2) -> f32>(
        f: F,
        grid_size: f32,
        n_steps: usize,
        texture: Texture2D,
    ) -> Self {
        let mut points = Vec::new();
        let mut tiles = Vec::new();
        let mut mesh = Mesh {
            vertices: Vec::new(),
            indices: Vec::new(),
            texture: Some(texture),
        };
        for iy in 0..(n_steps + 1) {
            for ix in 0..(n_steps + 1) {
                let uv = Vec2::new(ix as f32, iy as f32) / n_steps as f32;
                let coord = grid_size * (uv - 0.5);
                let point = sample_height_map(&f, coord);
                points.push(point);
                mesh.vertices.push(Vertex {
                    position: point.pos,
                    uv,
                    color: Color::from_vec(Vec4::from((Vec3::splat(point.normal.z), 1.0))).into(),
                    normal: Vec4::from((point.normal, 1.0)),
                });
                if ix != 0 && iy != 0 {
                    let n = points.len();
                    let square_indices = [n - 1, n - 2, n - n_steps - 2, n - n_steps - 3];
                    let new_tile_indices =
                        [[0, 1, 2], [1, 3, 2]].map(|ti| ti.map(|i| square_indices[i]));
                    tiles.extend(
                        new_tile_indices.map(|ti| Triangle3::from(ti.map(|i| points[i].pos))),
                    );
                    mesh.indices
                        .extend(new_tile_indices.into_iter().flatten().map(|i| i as u16));
                }
            }
        }
        Self { mesh, tiles }
    }

    pub fn tiles(&self) -> impl Iterator<Item = Triangle3> + '_ {
        self.tiles.iter().cloned()
    }

    /// Returns: (distance from start, intersection point, normal at the point)
    pub fn intersect_line(&self, start: Vec3, end: Vec3) -> Option<(f32, Vec3, Vec3)> {
        self.tiles()
            .filter_map(|tile| tile.intersect_line(start, end))
            .min_by(|(dist0, ..), (dist1, ..)| dist0.total_cmp(dist1))
    }

    pub fn draw(&self) {
        draw_mesh(&self.mesh);
    }
}

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
