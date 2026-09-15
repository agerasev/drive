use crate::geometry::Triangle3;
use glam::{Vec2, Vec3};

pub struct TerrainVertex {
    pub position: Vec3,
    pub uv: Vec2,
    pub shade: f32,
}
pub struct Terrain {
    pub vertices: Vec<TerrainVertex>,
    pub indices: Vec<u32>,
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
        let delta = 0.01;
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
    /// Dry friction coefficient.
    pub const DRY_FRICTION: f32 = 0.4;

    pub fn from_height_map<F: Fn(Vec2) -> f32>(f: F, grid_size: f32, n_steps: usize) -> Self {
        assert!(n_steps > 0 && grid_size.is_finite() && grid_size > 0.0);
        let mut points = Vec::new();
        let mut tiles = Vec::new();
        let mut vertices = Vec::new();
        let mut indices = Vec::new();
        for iy in 0..(n_steps + 1) {
            for ix in 0..(n_steps + 1) {
                let uv = Vec2::new(ix as f32, iy as f32) / n_steps as f32;
                let coord = grid_size * (uv - 0.5);
                let point = sample_height_map(&f, coord);
                points.push(point);
                vertices.push(TerrainVertex {
                    position: point.pos,
                    uv,
                    shade: point.normal.z,
                });
                if ix != 0 && iy != 0 {
                    let n = points.len();
                    let square_indices = [n - 1, n - 2, n - n_steps - 2, n - n_steps - 3];
                    let new_tile_indices =
                        [[0, 1, 2], [1, 3, 2]].map(|ti| ti.map(|i| square_indices[i]));
                    tiles.extend(
                        new_tile_indices.map(|ti| Triangle3::from(ti.map(|i| points[i].pos))),
                    );
                    indices.extend(new_tile_indices.into_iter().flatten().map(|i| i as u32));
                }
            }
        }
        Self {
            vertices,
            indices,
            tiles,
        }
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
}
