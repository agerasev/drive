use derive_more::derive::{Deref, DerefMut};
use macroquad::{
    color::Color,
    math::{Vec2, Vec3, Vec4},
    models::{draw_mesh, Mesh},
    ui::Vertex,
};

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Tile {
    pub vertices: [Vec3; 3],
}

impl Tile {
    pub fn normal(&self) -> Vec3 {
        (self.vertices[0] - self.vertices[1])
            .cross(self.vertices[2] - self.vertices[0])
            .normalize_or_zero()
    }
}

pub struct Terrain {
    mesh: Mesh,
    pub tiles: Vec<Tile>,
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
    pub fn from_height_map<F: Fn(Vec2) -> f32>(f: F, grid_size: f32, n_steps: usize) -> Self {
        let mut points = Vec::new();
        let mut tiles = Vec::new();
        let mut mesh = Mesh {
            vertices: Vec::new(),
            indices: Vec::new(),
            texture: None,
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
                    color: Color::from_vec(Vec4::from((uv, 1.0 - 0.5 * uv.element_sum(), 1.0)))
                        .into(),
                    normal: Vec4::from((point.normal, 1.0)),
                });
                if ix != 0 && iy != 0 {
                    let n = points.len();
                    let square_indices = [n - 1, n - 2, n - n_steps - 2, n - n_steps - 3];
                    let new_tile_indices =
                        [[0, 1, 2], [1, 3, 2]].map(|ti| ti.map(|i| square_indices[i]));
                    tiles.extend(new_tile_indices.map(|ti| Tile {
                        vertices: ti.map(|i| points[i].pos),
                    }));
                    mesh.indices
                        .extend(new_tile_indices.into_iter().flatten().map(|i| i as u16));
                }
            }
        }
        Self { mesh, tiles }
    }

    pub fn draw(&self) {
        draw_mesh(&self.mesh);
    }
}
