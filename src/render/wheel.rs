//! One outer spoke face, an open inner sidewall and a plain metal rim barrel.
use glam::{Vec2, Vec3, Vec4};
use wgame::{
    Library,
    prelude::*,
    shapes::{Mesh, PolygonFill, shader::Vertex},
};

const SECTORS: u32 = 32;
// Inner edge of the silver rim in the wheel's 256-pixel radial chart.
const RIM_RADIUS: f32 = 0.735;

/// Local +Z is the outside of the vehicle; both tire faces retain their sidewall.
pub(super) fn tire_geometry() -> (Vec<Vertex>, Vec<u32>) {
    let (mut vertices, indices) = wgame::shapes3d::cylinder_mesh(SECTORS);
    let mut indices: Vec<_> = indices
        .as_chunks::<3>()
        .0
        .iter()
        .filter(|tri| vertices[tri[0] as usize].normal.z > -0.5)
        .flatten()
        .copied()
        .collect();
    // The inner face is an annulus, not another disk of spokes.
    let start = vertices.len() as u32;
    for i in 0..=SECTORS {
        let direction = Vec2::from_angle(std::f32::consts::TAU * i as f32 / SECTORS as f32);
        for radius in [RIM_RADIUS, 1.0] {
            vertices.push(
                Vertex::new((direction * radius).extend(-0.5).extend(1.0), Vec3::ZERO)
                    .with_normal(-Vec3::Z),
            );
        }
        if i < SECTORS {
            let a = start + 2 * i;
            indices.extend([a, a + 2, a + 1, a + 1, a + 2, a + 3]);
        }
    }
    (vertices, indices)
}

pub(super) fn barrel(lib: &Library) -> PolygonFill {
    let (mut vertices, indices) = wgame::shapes3d::cylinder_mesh(SECTORS);
    let indices: Vec<_> = indices
        .as_chunks::<3>()
        .0
        .iter()
        .filter(|tri| vertices[tri[0] as usize].normal.z.abs() < 0.5)
        .flat_map(|tri| [tri[0], tri[2], tri[1]])
        .collect();
    for v in &mut vertices {
        v.pos.x *= RIM_RADIUS;
        v.pos.y *= RIM_RADIUS;
        v.normal = -v.normal;
    }
    lib.shapes()
        .mesh(Mesh::from_arrays(
            lib.shapes().state(),
            &vertices,
            Some(&indices),
        ))
        .fill_color(Vec4::new(0.22, 0.22, 0.22, 1.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inner_side_is_open_and_outer_spokes_have_only_one_plane() {
        let (vertices, indices) = tire_geometry();
        let mut inner_triangles = 0;
        let mut outer_triangles = 0;
        for tri in indices.as_chunks::<3>().0 {
            let vs = tri.map(|i| vertices[i as usize]);
            let [a, b, c] = vs.map(|v| v.pos.truncate());
            let normal = (b - a).cross(c - a);
            assert!(normal.dot(vs[0].normal) > 0.0);
            if vs[0].normal.z < -0.5 {
                inner_triangles += 1;
                for p in [a, b, c] {
                    assert!(p.truncate().length() >= RIM_RADIUS - 1e-6);
                }
            } else if vs[0].normal.z > 0.5 {
                outer_triangles += 1;
                assert!([a, b, c].iter().all(|p| p.z == 0.5));
            }
        }
        assert_eq!(inner_triangles, SECTORS * 2);
        assert_eq!(outer_triangles, SECTORS);
    }
}
