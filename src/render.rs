use drive::{terrain::Terrain, vehicle::Vehicle};
use glam::{Vec3, Vec4};
use rand::{RngExt, SeedableRng, rngs::SmallRng};
use std::io::Cursor;
use wgame::{
    Library, Result,
    gfx::types::{Color, color},
    gfx::{BakedScene, Scene},
    image::{Atlas, Image, ImageBase, ImageWriteMut},
    prelude::*,
    shapes::{Mesh, PolygonFill, shader::Vertex},
    shapes3d::{
        LightParameters, Lighting, LitMaterial, LitMesh, MaterialSettings, NormalSpace, NormalY,
        recalculate_normals,
    },
    texture::{Texture, TextureAtlas, TextureSettings},
};

pub struct Assets {
    cars: [Vec<LitMesh>; 2],
    wheel: LitMesh,
    lighting: Lighting,
    marker: PolygonFill,
    pub terrain: BakedScene,
}
fn texture(lib: &Library, bytes: &[u8]) -> Result<Texture> {
    let image = Image::decode_auto(bytes)?;
    let size = image.size();
    if size.width >= 2048 || size.height >= 2048 {
        // Atlas padding makes a 4K map exceed 4096. Keep large maps in exact-sized
        // allocations so packing several cannot grow the shared atlas to 16K.
        let atlas = TextureAtlas::new(
            lib.texturing().state(),
            Atlas::with_size((size.width + 2, size.height + 2).into()),
            wgpu::TextureFormat::Rgba16Float,
        );
        let texture = atlas.allocate(size, TextureSettings::linear());
        texture.update(|mut dst| dst.copy_from(&image));
        Ok(texture)
    } else {
        Ok(lib.make_texture(&image, TextureSettings::linear()))
    }
}
fn body_material(
    lighting: &Lighting,
    color: &Texture,
    normal: &Texture,
    normal_space: NormalSpace,
) -> Result<LitMaterial> {
    anyhow::ensure!(
        color.size() == normal.size(),
        "body color and normal map dimensions differ"
    );
    lighting.material(
        Some(normal),
        MaterialSettings {
            specular: 0.2,
            shininess: 48.0,
            normal_space,
            // For tangent-space maps, the OBJ exporter flips Blender's V for top-left image sampling.
            // Its baked +Y normals therefore point against increasing game V.
            normal_y: NormalY::Negative,
            ..Default::default()
        },
    )
}
fn model(
    lib: &Library,
    bytes: &[u8],
    texture: &Texture,
    material: &LitMaterial,
) -> Result<Vec<LitMesh>> {
    model_with_materials(lib, bytes, texture, |_| material)
}
fn model_with_materials<'a>(
    lib: &Library,
    bytes: &[u8],
    texture: &Texture,
    material: impl Fn(&str) -> &'a LitMaterial,
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
            let material = material(&m.name);
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
            let mesh = lib
                .shapes()
                .mesh(Mesh::from_arrays(
                    lib.shapes().state(),
                    &vertices,
                    Some(&mesh.indices),
                ))
                .fill_texture(texture)
                .with_material(material);
            Ok(mesh)
        })
        .collect()
}
fn detail_models(
    lib: &Library,
    lighting: &Lighting,
    obj: &[u8],
    png: &[u8],
) -> Result<Vec<LitMesh>> {
    let color = texture(lib, png)?;
    // Mirrors and windshield retain their own geometry and explicit normals.
    // Applying the body bake here would project through thin, separate surfaces.
    let material = lighting.material(
        None,
        MaterialSettings {
            specular: 0.2,
            shininess: 48.0,
            ..Default::default()
        },
    )?;
    let matte = lighting.material(
        None,
        MaterialSettings {
            specular: 0.0,
            ..Default::default()
        },
    )?;
    model_with_materials(lib, obj, &color, |name| {
        if name == "opaque_underbody" {
            &matte
        } else {
            &material
        }
    })
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
        let logan_normal = texture(lib, include_bytes!("../assets/logan/normal.png"))?;
        let l200_normal = texture(lib, include_bytes!("../assets/l200/normal.png"))?;
        let wheel = texture(lib, include_bytes!("../assets/wheel/color.png"))?;
        // Decode normal RGB as raw data. The lighting shader only linearizes albedo.
        // Wheel normal vectors are already rotated into the packed atlas UV frame.
        let wheel_normal = texture(lib, include_bytes!("../assets/wheel/normal.png"))?;
        anyhow::ensure!(
            wheel.size() == wheel_normal.size(),
            "wheel color and normal map dimensions differ"
        );
        let lighting = Lighting::new(lib.shapes(), lib.texturing(), LightParameters::default())?;
        let logan_material = body_material(&lighting, &logan, &logan_normal, NormalSpace::Object)?;
        let l200_material = body_material(&lighting, &l200, &l200_normal, NormalSpace::Object)?;
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
        let mut cars = [
            model(
                lib,
                include_bytes!("../assets/logan/model.obj"),
                &logan,
                &logan_material,
            )?,
            model(
                lib,
                include_bytes!("../assets/l200/model.obj"),
                &l200,
                &l200_material,
            )?,
        ];
        cars[0].extend(detail_models(
            lib,
            &lighting,
            include_bytes!("../assets/logan/details.obj"),
            include_bytes!("../assets/logan/details.png"),
        )?);
        cars[1].extend(detail_models(
            lib,
            &lighting,
            include_bytes!("../assets/l200/details.obj"),
            include_bytes!("../assets/l200/details.png"),
        )?);
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
