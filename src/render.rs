use drive::{terrain::Terrain, vehicle::Vehicle};
use glam::{Vec3, Vec4};
use rand::{Rng, SeedableRng, rngs::SmallRng};
use std::io::Cursor;
use wgame::{
    Library, Result,
    gfx::types::{Color, color},
    gfx::{BakedScene, Scene},
    image::Image,
    prelude::*,
    shapes::{Mesh, PolygonFill, shader::Vertex},
    texture::{Texture, TextureSettings},
};

pub struct Assets {
    cars: [Vec<PolygonFill>; 2],
    wheel: PolygonFill,
    marker: PolygonFill,
    pub terrain: BakedScene,
}
fn texture(lib: &Library, bytes: &[u8]) -> Result<Texture> {
    Ok(lib.make_texture(&Image::decode_auto(bytes)?, TextureSettings::linear()))
}
fn model(lib: &Library, bytes: &[u8], texture: &Texture) -> Result<Vec<PolygonFill>> {
    let (models, _) = tobj::load_obj_buf(
        &mut Cursor::new(bytes),
        &tobj::LoadOptions {
            single_index: true,
            triangulate: true,
            ignore_points: true,
            ignore_lines: true,
        },
        |_| Ok((Vec::new(), Default::default())),
    )?;
    anyhow::ensure!(!models.is_empty(), "OBJ contains no models");
    models
        .into_iter()
        .map(|m| {
            let mesh = m.mesh;
            anyhow::ensure!(
                !mesh.positions.is_empty() && mesh.positions.len().is_multiple_of(3),
                "invalid OBJ positions"
            );
            let vertices: Vec<_> = mesh
                .positions
                .as_chunks::<3>()
                .0
                .iter()
                .enumerate()
                .map(|(i, p)| {
                    let uv = if mesh.texcoords.len() >= 2 * i + 2 {
                        glam::Vec2::new(mesh.texcoords[2 * i], mesh.texcoords[2 * i + 1])
                    } else {
                        glam::Vec2::ZERO
                    };
                    Vertex::new(Vec3::from_array(*p).extend(1.0), uv.extend(1.0))
                })
                .collect();
            anyhow::ensure!(
                mesh.indices.iter().all(|&i| (i as usize) < vertices.len()),
                "OBJ index out of bounds"
            );
            anyhow::ensure!(
                vertices
                    .iter()
                    .all(|v| v.pos.is_finite() && v.local_coord.is_finite()),
                "non-finite OBJ vertex"
            );
            Ok(lib
                .shapes()
                .mesh(Mesh::from_arrays(
                    lib.shapes().state(),
                    &vertices,
                    Some(&mesh.indices),
                ))
                .fill_texture(texture))
        })
        .collect()
}
impl Assets {
    pub fn new(lib: &Library, terrain: &Terrain) -> Result<Self> {
        let logan = texture(lib, include_bytes!("../assets/logan/color.png"))?;
        let l200 = texture(lib, include_bytes!("../assets/l200/color.png"))?;
        let wheel = texture(lib, include_bytes!("../assets/wheel/color.png"))?;
        let mut rng = SmallRng::seed_from_u64(0xdeadbeef);
        let ground = lib.make_texture(
            &Image::with_data(
                (256, 256),
                (0..256 * 256)
                    .map(|_| {
                        let noise = 0.25 * rng.random::<f32>();
                        Vec4::new(noise, 0.5 + noise, noise, 1.0).to_rgba_f16()
                    })
                    .collect::<Vec<_>>(),
            ),
            TextureSettings::linear(),
        );
        let cars = [
            model(lib, include_bytes!("../assets/logan/model.obj"), &logan)?,
            model(lib, include_bytes!("../assets/l200/model.obj"), &l200)?,
        ];
        // The legacy wheel uses a radial projection into the first 256x256
        // region of a 320x256 texture, including the outer rim on the tread.
        let (mut wheel_vertices, wheel_indices) = wgame::shapes3d::cylinder_mesh(32);
        for vertex in &mut wheel_vertices {
            vertex.local_coord = glam::Vec2::new(
                (vertex.pos.x + 1.0) * 128.0 / wheel.size().width as f32,
                (vertex.pos.y + 1.0) * 128.0 / wheel.size().height as f32,
            )
            .extend(1.0);
        }
        let wheel = lib
            .shapes()
            .mesh(Mesh::from_arrays(
                lib.shapes().state(),
                &wheel_vertices,
                Some(&wheel_indices),
            ))
            .fill_texture(&wheel);
        let marker = lib.shapes().sphere(16, 8).fill_color(color::RED);
        let vertices: Vec<_> = terrain
            .vertices
            .iter()
            .map(|v| {
                Vertex::new(v.position.extend(1.0), v.uv.extend(1.0))
                    .with_color(Vec4::new(v.shade, v.shade, v.shade, 1.0))
            })
            .collect();
        let mut scene = Scene::default();
        scene.add(
            &lib.shapes()
                .mesh(Mesh::from_arrays(
                    lib.shapes().state(),
                    &vertices,
                    Some(&terrain.indices),
                ))
                .fill_texture(&ground),
        );
        // All assets exist before baking, so the atlas allocation is stable.
        Ok(Self {
            cars,
            wheel,
            marker,
            terrain: scene.bake(),
        })
    }
    pub fn draw_vehicle(&self, car: &Vehicle, model: usize, scene: &mut Scene) {
        for body in &self.cars[model] {
            scene.add(&body.transform(car.transform()));
        }
        for transform in car.wheel_transforms() {
            scene.add(&self.wheel.transform(transform));
        }
    }
    pub fn draw_marker(&self, position: Vec3, radius: f32, scene: &mut Scene) {
        scene.add(&self.marker.scale(radius).move_to(position));
    }
}

#[cfg(all(test, not(target_arch = "wasm32")))]
mod tests;
