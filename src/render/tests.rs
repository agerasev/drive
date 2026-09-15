use wgame::gfx::{Graphics, Offscreen, prelude::*};
fn graphics() -> Graphics {
    futures::executor::block_on(async {
        let instance =
            wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle_from_env());
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions::default())
            .await
            .expect("GPU adapter required (Mesa lavapipe works)");
        eprintln!("Adapter: {:?}", adapter.get_info());
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                required_limits: wgpu::Limits::downlevel_webgl2_defaults()
                    .using_resolution(adapter.limits()),
                ..Default::default()
            })
            .await
            .unwrap();
        Graphics::new(adapter, device, queue, wgpu::TextureFormat::Rgba8Unorm)
    })
}
fn pixels(target: &mut Offscreen) -> Vec<u8> {
    let (width, height) = target.size();
    let stride = (width * 4).div_ceil(256) * 256;
    let buffer = target
        .state()
        .device()
        .create_buffer(&wgpu::BufferDescriptor {
            label: Some("readback"),
            size: (stride * height) as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
    let texture = target.texture().clone();
    target.encoder().copy_texture_to_buffer(
        texture.as_image_copy(),
        wgpu::TexelCopyBufferInfo {
            buffer: &buffer,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(stride),
                rows_per_image: Some(height),
            },
        },
        wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
    );
    target.submit();
    let (tx, rx) = std::sync::mpsc::channel();
    buffer
        .slice(..)
        .map_async(wgpu::MapMode::Read, move |r| tx.send(r).unwrap());
    target
        .state()
        .device()
        .poll(wgpu::PollType::Wait {
            submission_index: None,
            timeout: Some(std::time::Duration::from_secs(30)),
        })
        .unwrap();
    rx.recv().unwrap().unwrap();
    let data = buffer
        .slice(..)
        .get_mapped_range()
        .expect("readback buffer must be mapped");
    let pixels = data
        .chunks(stride as usize)
        .flat_map(|row| row[..width as usize * 4].iter().copied())
        .collect();
    drop(data);
    buffer.unmap();
    pixels
}

#[test]
#[ignore = "requires a GPU adapter; run with --ignored"]
fn both_vehicle_assets_render_with_shared_depth() {
    use super::*;
    use crate::{camera::Orbit, config, spawn};
    use wgame::gfx::Camera;
    let gfx = graphics();
    let library = Library::new(&gfx);
    let terrain = Terrain::from_height_map(
        |c| 8.0 * (1.0 - 1.0 / (1.0 + 0.002 * c.length_squared())),
        64.0,
        24,
    );
    let assets = Assets::new(&library, &terrain).unwrap();
    for model in 0..2 {
        assert!(config(model).unwrap().mass > 0.0);
        let mut car = spawn(model).unwrap();
        for _ in 0..1440 {
            car.step(&terrain, 1.0 / 240.0);
        }
        for view in 0..2 {
            let mut orbit = Orbit::default();
            orbit.rotate(glam::Vec2::new(view as f32 * 1200.0, 0.0));
            let mut target = Offscreen::new(&gfx, (800, 450));
            let (matrix, eye) = orbit.view(car.pos(), &terrain, 800.0 / 450.0);
            assets.update_lighting(eye).unwrap();
            let camera = Camera::new(&gfx, matrix);
            let mut scene = Scene::default();
            assets.draw_vehicle(&car, model, &mut scene);
            target.clear(color::BLACK);
            target.render(&camera, &assets.terrain);
            let ground = pixels(&mut target);
            target.render_iter(&camera, scene.iter());
            let forward = pixels(&mut target);
            assert!(
                ground
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .filter(|p| p[1] > p[0] + 20)
                    .count()
                    > 100_000
            );
            assert!(
                forward
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .zip(ground.as_chunks::<4>().0.iter())
                    .filter(|(a, b)| a != b)
                    .count()
                    > 1000
            );
            // Separate passes must share depth, regardless of opaque draw order.
            target.clear(color::BLACK);
            target.render_iter(&camera, scene.iter());
            target.render(&camera, &assets.terrain);
            let backward = pixels(&mut target);
            let differences = forward
                .as_chunks::<4>()
                .0
                .iter()
                .zip(backward.as_chunks::<4>().0.iter())
                .filter(|(a, b)| a != b)
                .count();
            // Linear filtering creates a few translucent edge pixels. Their
            // blend color depends on what was drawn before the depth write;
            // the opaque interiors must remain order independent.
            assert!(
                differences <= 32,
                "model {model}, view {view}: draw order changed {differences} pixels"
            );
            save_image(&format!("drive-{model}-{view}"), target.size(), &forward);
        }
    }
}

fn save_image(name: &str, (width, height): (u32, u32), pixels: &[u8]) {
    if let Some(dir) = std::env::var_os("DRIVE_RENDER_OUTPUT") {
        use std::io::Write;
        let mut file =
            std::fs::File::create(std::path::Path::new(&dir).join(format!("{name}.ppm"))).unwrap();
        write!(file, "P6\n{width} {height}\n255\n").unwrap();
        for pixel in pixels.as_chunks::<4>().0 {
            file.write_all(&pixel[..3]).unwrap();
        }
    }
}

#[test]
#[ignore = "requires a GPU adapter; run with --ignored"]
fn wheel_normal_map_shades_face_and_tread() {
    use super::*;
    use glam::{Affine3A, Vec3};
    use wgame::gfx::Camera;
    let gfx = graphics();
    let lib = Library::new(&gfx);
    let color_map = texture(&lib, include_bytes!("../../assets/wheel/color.png")).unwrap();
    let normal_map = texture(&lib, include_bytes!("../../assets/wheel/normal.png")).unwrap();
    let eye = Vec3::new(2.0, -3.0, 3.0);
    let lighting = Lighting::new(
        lib.shapes(),
        lib.texturing(),
        LightParameters {
            direction: Vec3::new(-0.5, -1.0, 1.0),
            eye,
            ..Default::default()
        },
    )
    .unwrap();
    let view = glam::camera::rh::proj::directx::perspective(1.0, 1.0, 0.1, 20.0)
        * glam::camera::rh::view::look_at_mat4(eye, Vec3::ZERO, Vec3::Z);
    let camera = Camera::new(&gfx, view);
    let mut target = Offscreen::new(&gfx, (512, 512));
    let mesh =
        wheel_mesh(&lib, &color_map).transform(Affine3A::from_scale(Vec3::new(1.0, 1.0, 0.6)));
    let render = |target: &mut Offscreen, normal| {
        let material = lighting
            .material(
                normal,
                MaterialSettings {
                    specular: 0.08,
                    shininess: 24.0,
                    ..Default::default()
                },
            )
            .unwrap();
        let mut scene = Scene::default();
        scene.add(&mesh.with_material(&material));
        target.clear(color::BLACK);
        target.render(&camera, &scene.bake());
        pixels(target)
    };
    let flat = render(&mut target, None);
    let mapped = render(&mut target, Some(&normal_map));
    let changed = flat
        .as_chunks::<4>()
        .0
        .iter()
        .zip(mapped.as_chunks::<4>().0.iter())
        .filter(|(a, b)| a[..3].iter().zip(&b[..3]).any(|(a, b)| a.abs_diff(*b) > 5))
        .count();
    assert!(
        changed > 2000,
        "normal mapping changed only {changed} pixels"
    );
    save_image("wheel-flat", target.size(), &flat);
    save_image("wheel-normal", target.size(), &mapped);
}
