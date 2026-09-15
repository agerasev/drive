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
    shapes3d::{
        LightParameters, Lighting, LitMaterial, LitMesh, MaterialSettings, NormalY,
        recalculate_normals,
    },
    texture::{Texture, TextureSettings},
};

pub struct Assets {
    cars: [Vec<LitMesh>; 2],
    wheel: LitMesh,
    lighting: Lighting,
    marker: PolygonFill,
    pub terrain: BakedScene,
}
fn texture(lib: &Library, bytes: &[u8]) -> Result<Texture> {
    Ok(lib.make_texture(&Image::decode_auto(bytes)?, TextureSettings::linear()))
}
fn model(
    lib: &Library,
    bytes: &[u8],
    texture: &Texture,
    material: &LitMaterial,
) -> Result<Vec<LitMesh>> {
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
            let mut vertices: Vec<_> = mesh
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
            anyhow::ensure!(
                mesh.normals.is_empty() || mesh.normals.len() == vertices.len() * 3,
                "invalid OBJ normal count"
            );
            if !mesh.normals.is_empty() {
                for (vertex, normal) in vertices.iter_mut().zip(mesh.normals.as_chunks::<3>().0) {
                    vertex.normal = Vec3::from_array(*normal)
                        .try_normalize()
                        .ok_or_else(|| anyhow::anyhow!("invalid OBJ normal"))?;
                }
            } else {
                recalculate_normals(&mut vertices, &mesh.indices)?;
            }
            Ok(lib
                .shapes()
                .mesh(Mesh::from_arrays(
                    lib.shapes().state(),
                    &vertices,
                    Some(&mesh.indices),
                ))
                .fill_texture(texture)
                .with_material(material))
        })
        .collect()
}
fn wheel_mesh(lib: &Library, texture: &Texture) -> PolygonFill {
    // Both wheel maps have a radial face in the first 256x256 pixels and
    // a 64-pixel tread strip on the right. Keep separate cap/side UV charts.
    let (mut wheel_vertices, wheel_indices) = wgame::shapes3d::cylinder_mesh(32);
    for vertex in &mut wheel_vertices {
        let pixel = if vertex.normal.z.abs() > 0.5 {
            glam::Vec2::new(vertex.pos.x + 1.0, vertex.pos.y + 1.0) * 128.0
        } else {
            glam::Vec2::new(
                256.0 + 64.0 * vertex.local_coord.y,
                256.0 * vertex.local_coord.x,
            )
        };
        vertex.local_coord = (pixel
            / glam::Vec2::new(texture.size().width as f32, texture.size().height as f32))
        .extend(1.0);
    }
    lib.shapes()
        .mesh(Mesh::from_arrays(
            lib.shapes().state(),
            &wheel_vertices,
            Some(&wheel_indices),
        ))
        .fill_texture(texture)
}
impl Assets {
    pub fn new(lib: &Library, terrain: &Terrain) -> Result<Self> {
        let logan = texture(lib, include_bytes!("../assets/logan/color.png"))?;
        let l200 = texture(lib, include_bytes!("../assets/l200/color.png"))?;
        let wheel = texture(lib, include_bytes!("../assets/wheel/color.png"))?;
        // Decode normal RGB as raw data. The lighting shader only linearizes albedo.
        let wheel_normal = texture(lib, include_bytes!("../assets/wheel/normal.png"))?;
        anyhow::ensure!(
            wheel.size() == wheel_normal.size(),
            "wheel color and normal map dimensions differ"
        );
        let lighting = Lighting::new(lib.shapes(), lib.texturing(), LightParameters::default())?;
        let car_material = lighting.material(
            None,
            MaterialSettings {
                specular: 0.2,
                shininess: 48.0,
                ..Default::default()
            },
        )?;
        let wheel_material = lighting.material(
            Some(&wheel_normal),
            MaterialSettings {
                specular: 0.08,
                shininess: 24.0,
                normal_y: NormalY::Positive,
                ..Default::default()
            },
        )?;
        let ground_material = lighting.material(
            None,
            MaterialSettings {
                specular: 0.0,
                ..Default::default()
            },
        )?;
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
            model(
                lib,
                include_bytes!("../assets/logan/model.obj"),
                &logan,
                &car_material,
            )?,
            model(
                lib,
                include_bytes!("../assets/l200/model.obj"),
                &l200,
                &car_material,
            )?,
        ];
        let wheel = wheel_mesh(lib, &wheel).with_material(&wheel_material);
        let marker = lib.shapes().sphere(16, 8).fill_color(color::RED);
        let vertices: Vec<_> = terrain
            .vertices
            .iter()
            .map(|v| Vertex::new(v.position.extend(1.0), v.uv.extend(1.0)).with_normal(v.normal))
            .collect();
        let mut scene = Scene::default();
        scene.add(
            &lib.shapes()
                .mesh(Mesh::from_arrays(
                    lib.shapes().state(),
                    &vertices,
                    Some(&terrain.indices),
                ))
                .fill_texture(&ground)
                .with_material(&ground_material),
        );
        // All assets exist before baking, so the atlas allocation is stable.
        Ok(Self {
            cars,
            wheel,
            lighting,
            marker,
            terrain: scene.bake(),
        })
    }
    pub fn update_lighting(&self, eye: Vec3) -> Result<()> {
        self.lighting.update(LightParameters {
            eye,
            ..Default::default()
        })?;
        Ok(())
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
