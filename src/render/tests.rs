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
        for view in 0..4 {
            let mut orbit = Orbit::default();
            // Inspect both sides and wheel arches at a closer, lower angle.
            orbit.rotate(glam::Vec2::new((view as f32 + 0.5) * 785.0, -140.0));
            orbit.zoom(1.0);
            orbit.zoom(1.0);
            let mut target = Offscreen::new(&gfx, (800, 450));
            let (matrix, eye) = orbit.view(car.pos(), &terrain, 800.0 / 450.0);
            assets.update_lighting(eye).unwrap();
            let camera = Camera::new(&gfx, matrix);
            let mut scene = Scene::default();
            assets.draw_vehicle(
                &car,
                model,
                crate::appearance::linear_color(crate::appearance::DEFAULT_PAINT[model]),
                &mut scene,
            );
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
            let difference_budget: u32 = forward
                .as_chunks::<4>()
                .0
                .iter()
                .zip(backward.as_chunks::<4>().0.iter())
                .map(|(a, b)| {
                    a.iter()
                        .zip(b)
                        .map(|(a, b)| u32::from(a.abs_diff(*b)))
                        .max()
                        .unwrap()
                })
                .sum();
            // Filtered wheel edges contain fractional alpha. Weight differences
            // by magnitude so tiny edge blends do not count like opaque holes;
            // retain a budget equivalent to 32 fully different pixels.
            assert!(
                difference_budget <= 32 * 255,
                "model {model}, view {view}: draw order difference budget {difference_budget}"
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
fn both_body_maps_face_outward() {
    body_normals_face_outward(
        include_bytes!("../../assets/logan/model.obj"),
        include_bytes!("../../assets/logan/normal.png"),
    );
    body_normals_face_outward(
        include_bytes!("../../assets/l200/model.obj"),
        include_bytes!("../../assets/l200/normal.png"),
    );
}

fn body_normals_face_outward(obj: &[u8], png: &[u8]) {
    use std::io::Cursor;
    use wgame::image::{Image, ImageBase, ImageRead};

    let normal = Image::decode_auto(png).unwrap();
    let size = normal.size();
    let (parts, _) = tobj::load_obj_buf(
        &mut Cursor::new(obj),
        &tobj::LoadOptions {
            single_index: true,
            triangulate: true,
            ..Default::default()
        },
        |_| Ok((Vec::new(), Default::default())),
    )
    .unwrap();
    for part in parts {
        let mesh = part.mesh;
        let mut area = 0.0;
        let mut inward = 0.0;
        for triangle in mesh.indices.as_chunks::<3>().0 {
            let [a, b, c] = triangle.map(|i| {
                glam::Vec3::from_slice(&mesh.positions[3 * i as usize..3 * i as usize + 3])
            });
            let weight = (b - a).cross(c - a).length();
            let uv = triangle
                .iter()
                .map(|&i| {
                    glam::Vec2::from_slice(&mesh.texcoords[2 * i as usize..2 * i as usize + 2])
                })
                .sum::<glam::Vec2>()
                / 3.0;
            let x = ((uv.x * size.width as f32) as u32).min(size.width - 1);
            let y = ((uv.y * size.height as f32) as u32).min(size.height - 1);
            area += weight;
            let sample = normal.data()[(y * size.width + x) as usize];
            let mapped = glam::Vec3::new(sample.r.to_f32(), sample.g.to_f32(), sample.b.to_f32())
                * 2.0
                - glam::Vec3::ONE;
            if mapped.dot((b - a).cross(c - a)) < 0.0 {
                inward += weight;
            }
        }
        // Tiny grazing bevels can differ from the geometric normal. Whole inward
        // panels cannot: they shade black even though a two-sided source render
        // can appear correct. Weight by area so small trim does not hide this.
        assert!(
            area > 0.0 && inward / area < 0.05,
            "{}: {:.1}% of the panel area has inward baked normals",
            part.name,
            100.0 * inward / area
        );
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

#[test]
#[ignore = "requires a GPU adapter; run with --ignored"]
fn wheel_sidewall_lighting_stays_under_a_fixed_light_while_spinning() {
    use super::*;
    use glam::{Affine3A, Quat, Vec3};
    use wgame::gfx::Camera;
    let gfx = graphics();
    let lib = Library::new(&gfx);
    let normal = texture(&lib, include_bytes!("../../assets/wheel/normal.png")).unwrap();
    // Remove painted highlights: measure only the normal-map response.
    let white = lib.make_texture(
        &Image::with_color(normal.size(), color::WHITE.to_rgba_f16()),
        TextureSettings::linear(),
    );
    let lighting = Lighting::new(
        lib.shapes(),
        lib.texturing(),
        LightParameters {
            ambient: Vec3::ZERO,
            color: Vec3::ONE,
            direction: Vec3::Z,
            ..Default::default()
        },
    )
    .unwrap();
    let material = lighting
        .material(
            Some(&normal),
            MaterialSettings {
                specular: 0.,
                normal_y: NormalY::Positive,
                ..Default::default()
            },
        )
        .unwrap();
    let wheel = wheel_mesh(&lib, &white).with_material(&material);
    let mut target = Offscreen::new(&gfx, (256, 256));
    for side in [-1., 1.] {
        let eye = Vec3::new(side * 4., 0., 0.);
        let view = glam::camera::rh::proj::directx::perspective(0.65, 1., 0.1, 20.)
            * glam::camera::rh::view::look_at_mat4(eye, Vec3::ZERO, Vec3::Z);
        let camera = Camera::new(&gfx, view);
        for angle in [0., 0.4, 0.9, 1.6, 2.2, 3.] {
            let transform = Affine3A::from_scale_rotation_translation(
                Vec3::new(1., 1., 0.6),
                Quat::from_rotation_y(std::f32::consts::FRAC_PI_2) * Quat::from_rotation_z(angle),
                Vec3::ZERO,
            );
            let mut scene = Scene::default();
            scene.add(&wheel.transform(transform));
            target.clear(color::BLACK);
            target.render(&camera, &scene.bake());
            let data = pixels(&mut target);
            let brightness = |z| {
                let q = view * Vec3::new(side * 0.3, 0., z).extend(1.);
                let x = ((q.x / q.w * 0.5 + 0.5) * 256.) as usize;
                let y = ((0.5 - q.y / q.w * 0.5) * 256.) as usize;
                let mut sum = 0u32;
                for yy in y - 2..=y + 2 {
                    for xx in x - 2..=x + 2 {
                        sum += u32::from(data[(yy * 256 + xx) * 4]);
                    }
                }
                sum as f32 / 25.
            };
            let top = brightness(0.875);
            let bottom = brightness(-0.875);
            assert!(
                top > 120. && top > bottom + 80.,
                "wheel highlight follows spin instead of light: face {side}, angle {angle}, top {top}, bottom {bottom}"
            );
            save_image(
                &format!("wheel-fixed-light-{side}-{angle}"),
                target.size(),
                &data,
            );
        }
    }
}

#[test]
#[ignore = "requires a GPU adapter; run with --ignored"]
fn baked_body_normals_shade_both_vehicles() {
    use super::*;
    use wgame::gfx::Camera;

    let gfx = graphics();
    let lib = Library::new(&gfx);
    let eye = Vec3::new(5.0, 7.0, 3.5);
    let lighting = Lighting::new(
        lib.shapes(),
        lib.texturing(),
        LightParameters {
            eye,
            ..Default::default()
        },
    )
    .unwrap();
    let camera = Camera::new(
        &gfx,
        glam::camera::rh::proj::directx::perspective(0.8, 1.5, 0.1, 30.0)
            * glam::camera::rh::view::look_at_mat4(eye, Vec3::ZERO, Vec3::Z),
    );
    let mut target = Offscreen::new(&gfx, (900, 600));
    for (name, obj, albedo, normals, space) in [
        (
            "logan",
            &include_bytes!("../../assets/logan/model.obj")[..],
            &include_bytes!("../../assets/logan/color.png")[..],
            &include_bytes!("../../assets/logan/normal.png")[..],
            NormalSpace::Object,
        ),
        (
            "l200",
            &include_bytes!("../../assets/l200/model.obj")[..],
            &include_bytes!("../../assets/l200/color.png")[..],
            &include_bytes!("../../assets/l200/normal.png")[..],
            NormalSpace::Object,
        ),
    ] {
        let albedo = texture(&lib, albedo).unwrap();
        let normals = texture(&lib, normals).unwrap();
        let mapped = body_material(&lighting, &albedo, &normals, space).unwrap();
        let flat = lighting
            .material(
                None,
                MaterialSettings {
                    specular: 0.2,
                    shininess: 48.0,
                    ..Default::default()
                },
            )
            .unwrap();
        let render = |target: &mut Offscreen, material| {
            let mut scene = Scene::default();
            for mesh in model(&lib, obj, &albedo, material).unwrap() {
                scene.add(&mesh);
            }
            target.clear(color::BLACK);
            target.render(&camera, &scene.bake());
            pixels(target)
        };
        let flat_pixels = render(&mut target, &flat);
        let mapped_pixels = render(&mut target, &mapped);
        let changed = flat_pixels
            .as_chunks::<4>()
            .0
            .iter()
            .zip(mapped_pixels.as_chunks::<4>().0.iter())
            .filter(|(a, b)| a[..3].iter().zip(&b[..3]).any(|(a, b)| a.abs_diff(*b) > 5))
            .count();
        assert!(
            changed > 2000,
            "{name}: normal map changed only {changed} pixels"
        );
        save_image(&format!("{name}-flat"), target.size(), &flat_pixels);
        save_image(&format!("{name}-normal"), target.size(), &mapped_pixels);
    }
}

#[test]
#[ignore = "requires a GPU adapter; run with --ignored"]
fn guided_shell_close_views() {
    close_views(
        "logan",
        include_bytes!("../../assets/logan/model.obj"),
        include_bytes!("../../assets/logan/color.png"),
        include_bytes!("../../assets/logan/normal.png"),
    );
    close_views(
        "l200",
        include_bytes!("../../assets/l200/model.obj"),
        include_bytes!("../../assets/l200/color.png"),
        include_bytes!("../../assets/l200/normal.png"),
    );
}

fn close_views(car: &str, obj: &[u8], color: &[u8], normal: &[u8]) {
    use super::*;
    use wgame::gfx::Camera;
    let gfx = graphics();
    let lib = Library::new(&gfx);
    let albedo = texture(&lib, color).unwrap();
    let paint: &[u8] = if car == "logan" {
        include_bytes!("../../assets/logan/paint.png")
    } else {
        include_bytes!("../../assets/l200/paint.png")
    };
    let normals = normal_paint_texture(&lib, Some(normal), paint).unwrap();
    let paint_color = crate::appearance::linear_color(
        crate::appearance::DEFAULT_PAINT[usize::from(car == "l200")],
    );
    let lighting =
        Lighting::new(lib.shapes(), lib.texturing(), LightParameters::default()).unwrap();
    let material = body_material(&lighting, &albedo, &normals, NormalSpace::Object).unwrap();
    let mut scene = Scene::default();
    for mesh in model(&lib, obj, &albedo, &material).unwrap() {
        scene.add(&mesh.multiply_color(paint_color.extend(1.0)));
    }
    let (detail_obj, detail_png, detail_paint): (&[u8], &[u8], &[u8]) = if car == "logan" {
        (
            include_bytes!("../../assets/logan/details.obj"),
            include_bytes!("../../assets/logan/details.png"),
            include_bytes!("../../assets/logan/details-paint.png"),
        )
    } else {
        (
            include_bytes!("../../assets/l200/details.obj"),
            include_bytes!("../../assets/l200/details.png"),
            include_bytes!("../../assets/l200/details-paint.png"),
        )
    };
    for mesh in detail_models(&lib, &lighting, detail_obj, detail_png, detail_paint).unwrap() {
        scene.add(&mesh.multiply_color(paint_color.extend(1.0)));
    }
    let wheel_color = texture(&lib, include_bytes!("../../assets/wheel/color.png")).unwrap();
    let wheel_normal = texture(&lib, include_bytes!("../../assets/wheel/normal.png")).unwrap();
    let wheel_material = lighting
        .material(
            Some(&wheel_normal),
            MaterialSettings {
                normal_y: NormalY::Positive,
                specular: 0.08,
                shininess: 24.0,
                ..Default::default()
            },
        )
        .unwrap();
    let wheel = wheel_mesh(&lib, &wheel_color).with_material(&wheel_material);
    let terrain = Terrain::from_height_map(|_| 0.0, 32., 8);
    let mut vehicle = crate::spawn(usize::from(car == "l200")).unwrap();
    for _ in 0..1440 {
        vehicle.step(&terrain, 1.0 / 240.0);
    }
    let wheels: Vec<_> = vehicle
        .wheel_transforms()
        .into_iter()
        .map(|t| wheel.transform(vehicle.transform().inverse() * t))
        .collect();
    let mut wheel_scene = Scene::default();
    for wheel in &wheels {
        wheel_scene.add(wheel);
    }
    let wheel_scene = wheel_scene.bake();
    for (name, eye, aim) in [
        (
            "front-corner-low-wheel",
            Vec3::new(2.6, 4., -0.55),
            Vec3::new(0.6, 1.98, -0.15),
        ),
        (
            "rear-corner-low-wheel",
            Vec3::new(-2.6, -4., -0.55),
            Vec3::new(-0.6, -1.98, -0.15),
        ),
        (
            "front-arch-wheel",
            Vec3::new(2.8, if car == "l200" { 2.1 } else { 1.7 }, 0.03),
            Vec3::new(0.9, if car == "l200" { 1.97 } else { 1.48 }, 0.07),
        ),
        (
            "front-arch",
            Vec3::new(2.8, if car == "l200" { 2.1 } else { 1.7 }, 0.03),
            Vec3::new(0.9, if car == "l200" { 1.97 } else { 1.48 }, 0.07),
        ),
        (
            "rear-arch-wheel",
            Vec3::new(2.8, -1.7, 0.03),
            Vec3::new(0.9, -1.36, 0.0),
        ),
        (
            "handles",
            Vec3::new(3.5, 0.2, 0.1),
            Vec3::new(1.1, 0.0, 0.07),
        ),
        (
            "cab-join",
            Vec3::new(3., -2., 0.4),
            Vec3::new(1., -0.96, 0.14),
        ),
        (
            "bed-front-corner-left",
            Vec3::new(-3.0, 0.2, -0.20),
            Vec3::new(-1.05, -0.82, -0.48),
        ),
        (
            "bed-front-corner-right",
            Vec3::new(3.0, 0.2, -0.20),
            Vec3::new(1.05, -0.82, -0.48),
        ),
        (
            "rear-window",
            Vec3::new(0.25, -3.2, 2.7),
            Vec3::new(0., -1.4, 0.82),
        ),
        (
            "rear-roof-edge",
            Vec3::new(2.15, -1.7, 2.2),
            Vec3::new(0.75, -0.85, 1.03),
        ),
        (
            "underside",
            Vec3::new(2., -3., -3.3),
            Vec3::new(0., 0., -0.35),
        ),
        (
            "pillar-front",
            if car == "l200" {
                Vec3::new(0., 4.0, 1.8)
            } else {
                Vec3::new(0., 3.3, 1.8)
            },
            if car == "l200" {
                Vec3::new(0., 0.95, 0.55)
            } else {
                Vec3::new(0., 0.65, 0.65)
            },
        ),
        (
            "rear-whole",
            Vec3::new(0., -6., 1.8),
            Vec3::new(0., -0.7, -0.03),
        ),
        (
            "windshield",
            Vec3::new(1.9, 3.1, 1.9),
            Vec3::new(0., 0.9, 0.6),
        ),
        (
            "mirror",
            Vec3::new(2.0, 1.9, 1.3),
            Vec3::new(1.05, 0.85, 0.55),
        ),
        (
            "bed",
            Vec3::new(-3.5, -4.5, 3.0),
            Vec3::new(0., -1.6, -0.05),
        ),
        ("front", Vec3::new(3.1, 3.0, 1.6), Vec3::new(0.7, 1.1, 0.35)),
        (
            "rear",
            Vec3::new(-3.1, -3.0, 1.6),
            Vec3::new(-0.7, -1.1, 0.35),
        ),
        ("doors", Vec3::new(3.3, 0.3, 1.2), Vec3::new(0.9, 0.0, 0.4)),
        (
            "front-low",
            Vec3::new(0.15, 4.8, -0.10),
            Vec3::new(0.0, 2.0, 0.05),
        ),
        (
            "rear-low",
            Vec3::new(-0.15, -4.8, -0.12),
            Vec3::new(0.0, -2.0, 0.08),
        ),
        (
            "rear-low-wheel",
            Vec3::new(-0.15, -4.8, -0.12),
            Vec3::new(0.0, -2.0, 0.08),
        ),
    ] {
        lighting
            .update(LightParameters {
                eye,
                direction: Vec3::new(0.4, -0.8, 1.0),
                ..Default::default()
            })
            .unwrap();
        let matrix = glam::camera::rh::proj::directx::perspective(0.7, 1.25, 0.1, 30.0)
            * glam::camera::rh::view::look_at_mat4(eye, aim, Vec3::Z);
        let camera = Camera::new(&gfx, matrix);
        let mut target = Offscreen::new(&gfx, (1250, 1000));
        target.clear(Vec4::new(0.18, 0.18, 0.18, 1.0));
        target.render(&camera, &scene.bake());
        if name.ends_with("-wheel") {
            target.render(&camera, &wheel_scene);
        }
        let rendered = pixels(&mut target);
        if car == "l200" && name == "underside" {
            let center = (500 * 1250 + 625) * 4;
            assert_eq!(
                &rendered[center..center + 3],
                &[0, 0, 0],
                "floor must stay matte black"
            );
        }
        assert!(rendered.as_chunks::<4>().0.iter().any(|p| p[0] > 100));
        save_image(&format!("{car}-close-{name}"), target.size(), &rendered);
    }
}

#[test]
fn both_end_caps_follow_the_outer_envelope() {
    end_caps_follow_the_outer_envelope(
        include_bytes!("../../assets/logan/model.obj"),
        1.939,
        2.5,
        10,
        true,
    );
    end_caps_follow_the_outer_envelope(
        include_bytes!("../../assets/l200/model.obj"),
        2.549,
        3.05,
        9,
        false,
    );
}

fn end_caps_follow_the_outer_envelope(
    obj: &[u8],
    min: f32,
    max: f32,
    row_count: usize,
    rear: bool,
) {
    use std::{collections::BTreeMap, io::Cursor};
    let (parts, _) = tobj::load_obj_buf(
        &mut Cursor::new(obj),
        &tobj::LoadOptions {
            single_index: true,
            triangulate: true,
            ..Default::default()
        },
        |_| Ok((Vec::new(), Default::default())),
    )
    .unwrap();
    // The two end-cap atlas rectangles identify these faces even when a bad
    // projection puts their vertices deep inside the car. Position-only region
    // checks would silently miss the very vertices this regression targets.
    for (sign, v_min, v_max) in [(1.0_f32, 0.39, 0.67), (-1.0, 0.705, 0.985)] {
        if sign < 0.0 && !rear {
            continue; // The L200 rear is now authored geometry with its own UVs.
        }
        let mut rows: BTreeMap<i32, BTreeMap<i32, f32>> = BTreeMap::new();
        for part in &parts {
            let mesh = &part.mesh;
            for (i, uv) in mesh.texcoords.as_chunks::<2>().0.iter().enumerate() {
                let [mut u, mut v] = *uv;
                if part.name == "l200_bed" {
                    continue;
                }
                if part.name == "l200_cab" {
                    u = (u - 0.01) / 0.64;
                    v = (v - 0.02) / 0.96;
                }
                if !(0.835 - 1e-5..=0.985 + 1e-5).contains(&u)
                    || !(v_min - 1e-5..=v_max + 1e-5).contains(&v)
                {
                    continue;
                }
                let depth = sign * mesh.positions[3 * i + 1];
                assert!(
                    (min..=max).contains(&depth),
                    "end {sign}: vertex projected into car at depth {depth}"
                );
                rows.entry((v * 1e6).round() as i32)
                    .or_default()
                    .insert((u * 1e6).round() as i32, depth);
            }
        }
        assert!(rows.len() >= row_count, "end-cap UV rows missing");
        for row in rows.values() {
            let samples: Vec<_> = row.iter().collect();
            assert!(samples.len() >= 20, "end-cap UV columns missing");
            for three in samples.windows(3) {
                let [(u0, d0), (u1, d1), (u2, d2)] = three else {
                    unreachable!()
                };
                let t = (**u1 - **u0) as f32 / (**u2 - **u0) as f32;
                let chord = **d0 + (**d2 - **d0) * t;
                assert!(
                    **d1 >= chord - 0.002,
                    "end {sign}: inward fold of {}m",
                    chord - **d1
                );
            }
        }
    }
}

#[test]
fn l200_has_a_full_bumper_and_open_cargo_bed() {
    let (parts, _) = tobj::load_obj_buf(
        &mut std::io::Cursor::new(include_bytes!("../../assets/l200/model.obj")),
        &tobj::LoadOptions {
            single_index: true,
            ..Default::default()
        },
        |_| Ok((Vec::new(), Default::default())),
    )
    .unwrap();
    let vertices: Vec<_> = parts
        .iter()
        .flat_map(|p| p.mesh.positions.as_chunks::<3>().0)
        .collect();
    // The end grid must reach below the fog lamps, beyond the front wheel arch.
    assert!(
        vertices
            .iter()
            .any(|p| p[0].abs() < 0.4 && p[1] > 2.65 && p[2] < -0.70)
    );
    // A roof-style convex envelope would seal the pickup's cargo cavity.
    assert!(
        vertices
            .iter()
            .filter(|p| p[0].abs() < 0.4
                && (-2.3..-1.6).contains(&p[1])
                && (-0.35..-0.20).contains(&p[2]))
            .count()
            >= 10
    );
}

#[test]
fn source_details_have_consistent_normals_and_complete_mirrors() {
    for obj in [
        include_bytes!("../../assets/logan/details.obj").as_slice(),
        include_bytes!("../../assets/l200/details.obj").as_slice(),
    ] {
        let (parts, _) = tobj::load_obj_buf(
            &mut std::io::Cursor::new(obj),
            &tobj::LoadOptions {
                single_index: true,
                triangulate: true,
                ..Default::default()
            },
            |_| Ok((Vec::new(), Default::default())),
        )
        .unwrap();
        assert_eq!(
            parts
                .iter()
                .filter(|p| p.name.starts_with("mirror_"))
                .count(),
            2
        );
        for part in parts {
            let mesh = part.mesh;
            assert!(
                mesh.indices.len() >= 300,
                "{}: detailed surface missing",
                part.name
            );
            for tri in mesh.indices.as_chunks::<3>().0 {
                let points = tri.map(|i| {
                    glam::Vec3::from_slice(&mesh.positions[3 * i as usize..3 * i as usize + 3])
                });
                let normal = tri
                    .map(|i| {
                        glam::Vec3::from_slice(&mesh.normals[3 * i as usize..3 * i as usize + 3])
                    })
                    .into_iter()
                    .sum::<glam::Vec3>();
                assert!(normal.is_finite() && normal.length_squared() > 0.01);
                let geometric = (points[1] - points[0]).cross(points[2] - points[0]);
                assert!(
                    normal.dot(geometric) >= -1e-6,
                    "{}: inward detail normals",
                    part.name
                );
            }
            if part.name == "windshield_and_frame" {
                let mut normals = std::collections::BTreeMap::new();
                for (i, uv) in mesh.texcoords.as_chunks::<2>().0.iter().enumerate() {
                    // The glass swatch has constant UVs; the frame uses distinct swatches.
                    if (uv[0] - 0.53125).abs() > 1e-5 || (uv[1] - 0.96875).abs() > 1e-5 {
                        continue;
                    }
                    let p = &mesh.positions[3 * i..3 * i + 3];
                    let key = p
                        .iter()
                        .map(|c| (c * 1e5).round() as i32)
                        .collect::<Vec<_>>();
                    let n = glam::Vec3::from_slice(&mesh.normals[3 * i..3 * i + 3]);
                    if let Some(previous) = normals.insert(key, n) {
                        assert!(
                            n.dot(previous) > 0.999,
                            "glass shading changes across a triangle edge"
                        );
                    }
                }
                assert!(normals.len() > 100, "continuous windshield missing");
            }
        }
    }
}

#[test]
fn l200_rear_stops_at_the_authored_bumper() {
    let load = |obj: &[u8]| {
        tobj::load_obj_buf(
            &mut std::io::Cursor::new(obj),
            &tobj::LoadOptions {
                single_index: true,
                ..Default::default()
            },
            |_| Ok((Vec::new(), Default::default())),
        )
        .unwrap()
        .0
    };
    let body = load(include_bytes!("../../assets/l200/model.obj"));
    let rear = body
        .iter()
        .find(|p| p.name == "l200_bed")
        .expect("bed missing");
    let positions: Vec<_> = rear
        .mesh
        .positions
        .as_chunks::<3>()
        .0
        .iter()
        .filter(|p| p[1] < -2.40)
        .collect();
    assert!(!positions.is_empty());
    assert!(
        positions.iter().all(|p| p[2] > -0.61),
        "rear face extends below the source bumper"
    );
    assert!(positions.iter().any(|p| p[0] < -0.95));
    assert!(positions.iter().any(|p| p[0] > 0.95));
}

#[test]
fn pillars_keep_longitudinal_edges_on_the_source_ridges() {
    use glam::Vec3;
    // Source measurements along the sloping outer pillars. Nearby surface
    // triangles alone are insufficient: their edges must follow the ridge,
    // otherwise the silhouette steps between successive cross-sections.
    for (obj, samples, tolerance) in [
        (
            include_bytes!("../../assets/logan/model.obj").as_slice(),
            &[
                [0.74106, -0.975, 1.01192],
                [0.73657, -0.9, 1.03104],
                [0.73433, -0.85, 1.03909],
                [0.73194, -0.8, 1.04462],
                [0.78, 0.4, 0.94233],
                [0.8, 0.5, 0.88747],
                [0.82, 0.6, 0.82976],
                [0.84, 0.7, 0.76825],
                [0.86, 0.8, 0.70091],
                [0.88, 0.9, 0.63265],
            ][..],
            0.005,
        ),
        (
            include_bytes!("../../assets/l200/model.obj").as_slice(),
            &[[0.8075, 0.9, 0.61695], [0.8175, 1.0, 0.56046]][..],
            0.015,
        ),
    ] {
        let (parts, _) = tobj::load_obj_buf(
            &mut std::io::Cursor::new(obj),
            &tobj::LoadOptions {
                single_index: true,
                triangulate: true,
                ..Default::default()
            },
            |_| Ok((Vec::new(), Default::default())),
        )
        .unwrap();
        let mut edges = Vec::new();
        for part in parts {
            let mesh = part.mesh;
            for tri in mesh.indices.as_chunks::<3>().0 {
                let p = tri
                    .map(|i| Vec3::from_slice(&mesh.positions[3 * i as usize..3 * i as usize + 3]));
                for (a, b) in [(p[0], p[1]), (p[1], p[2]), (p[2], p[0])] {
                    let d = b - a;
                    if d.y.abs() > 1e-4
                        && d.x.abs() < 0.4 * d.y.abs()
                        && d.z.abs() < 1.2 * d.y.abs()
                    {
                        edges.push((a, d));
                    }
                }
            }
        }
        for sign in [-1.0, 1.0] {
            for p in samples {
                let point = Vec3::new(sign * p[0], p[1], p[2]);
                let error = edges
                    .iter()
                    .map(|(a, d)| {
                        let t = ((point - a).dot(*d) / d.length_squared()).clamp(0.0, 1.0);
                        (a + t * d).distance(point)
                    })
                    .fold(f32::INFINITY, f32::min);
                assert!(
                    error < tolerance,
                    "pillar ridge at {point:?} misses longitudinal edges by {error}m"
                );
            }
        }
    }
}

fn asset_parts(bytes: &[u8]) -> Vec<tobj::Model> {
    tobj::load_obj_buf(
        &mut std::io::Cursor::new(bytes),
        &tobj::LoadOptions {
            single_index: true,
            triangulate: true,
            ..Default::default()
        },
        |_| Ok((Vec::new(), Default::default())),
    )
    .unwrap()
    .0
}

#[test]
fn logan_rear_glass_normals_are_continuous_across_the_center() {
    use glam::{Vec2, Vec3};
    use wgame::image::{Image, ImageBase, ImageRead};
    let parts = asset_parts(include_bytes!("../../assets/logan/model.obj"));
    let mesh = &parts[0].mesh;
    let image = Image::decode_auto(include_bytes!("../../assets/logan/normal.png")).unwrap();
    let sample = |x, y| {
        let mut nearest = None;
        for tri in mesh.indices.as_chunks::<3>().0 {
            let p =
                tri.map(|i| Vec3::from_slice(&mesh.positions[3 * i as usize..3 * i as usize + 3]));
            let [a, b, c] = p.map(|v| Vec2::new(v.x, v.y));
            let det = (b - a).perp_dot(c - a);
            if det.abs() < 1e-8 {
                continue;
            }
            let q = Vec2::new(x, y) - a;
            let u = q.perp_dot(c - a) / det;
            let v = (b - a).perp_dot(q) / det;
            if u < 0. || v < 0. || u + v > 1. {
                continue;
            }
            let weights = [1. - u - v, u, v];
            let z = (0..3).map(|i| weights[i] * p[i].z).sum::<f32>();
            let uv = (0..3)
                .map(|i| {
                    let j = tri[i] as usize * 2;
                    Vec2::from_slice(&mesh.texcoords[j..j + 2]) * weights[i]
                })
                .sum::<Vec2>();
            if nearest.is_none_or(|(height, _)| z > height) {
                nearest = Some((z, uv));
            }
        }
        let (z, uv) = nearest.expect("rear glass missing");
        assert!(z > 0.6);
        let size = image.size();
        let n = image.data()[((uv.y * size.height as f32) as u32 * size.width
            + (uv.x * size.width as f32) as u32) as usize];
        (Vec3::new(n.r.to_f32(), n.g.to_f32(), n.b.to_f32()) * 2. - Vec3::ONE).normalize()
    };
    for y in [-1.25, -1.4, -1.55] {
        let left = sample(-0.005, y);
        let right = sample(0.005, y);
        assert!(
            left.distance(right) < 0.02,
            "rear glass normal seam at {y}: {left:?}, {right:?}"
        );
        assert!(left.y < -0.4 && left.z > 0.8, "glass lost its source slope");
    }
}

#[test]
fn logan_floor_closures_stay_above_the_bumper_outline() {
    use glam::Vec3;
    let parts = asset_parts(include_bytes!("../../assets/logan/model.obj"));
    let mesh = &parts[0].mesh;
    // Recess the floor without clipping the visible lower bumper panels.
    for (sign, z, minimum_extent) in [(1., -0.28, 2.12), (-1., -0.21, 1.98)] {
        let t = mesh_ray(mesh, Vec3::new(0., sign * 4., z), Vec3::new(0., -sign, 0.))
            .expect("lower bumper panel missing");
        assert!(
            4. - t > minimum_extent,
            "lower bumper clipped on side {sign}"
        );
    }
    for p in mesh.positions.as_chunks::<3>().0 {
        if p[1] < -1.94 {
            assert!(p[2] > -0.251, "rear closure hangs below bumper: {p:?}");
        }
        if p[1] > 1.94 {
            assert!(p[2] > -0.310, "front closure hangs below bumper: {p:?}");
        }
    }
    for x in [-0.5, 0., 0.5] {
        for y in [-1.9, -1.8, 0., 1.9] {
            let t = mesh_ray(mesh, Vec3::new(x, y, -1.), Vec3::Z)
                .expect("underside must remain closed");
            assert!((0.79..0.81).contains(&t), "floor misplaced at {x},{y}: {t}");
        }
    }
}

#[test]
fn logan_rear_apron_has_a_smooth_symmetric_lower_rim() {
    let parts = asset_parts(include_bytes!("../../assets/logan/model.obj"));
    let mesh = &parts[0].mesh;
    let mut rim = std::collections::BTreeMap::new();
    for (i, uv) in mesh.texcoords.as_chunks::<2>().0.iter().enumerate() {
        if (uv[1] - 0.985).abs() < 1e-6 && (0.83499..=0.98501).contains(&uv[0]) {
            rim.insert(
                (uv[0] * 1e6).round() as i32,
                &mesh.positions[3 * i..3 * i + 3],
            );
        }
    }
    let points: Vec<_> = rim.values().collect();
    assert_eq!(points.len(), 33);
    for (p, opposite) in points.iter().zip(points.iter().rev()) {
        assert!(
            (-0.251..=-0.243).contains(&p[2]),
            "notched rear apron: {p:?}"
        );
        assert!((p[0] + opposite[0]).abs() < 0.003);
        assert!((p[1] - opposite[1]).abs() < 0.003);
        assert!((p[2] - opposite[2]).abs() < 0.001);
    }
    for pair in points.windows(2) {
        assert!((pair[0][2] - pair[1][2]).abs() < 0.002, "abrupt apron step");
    }
}

#[test]
fn logan_bumper_corners_follow_source_hems_without_hanging_tabs() {
    use glam::Vec3;
    let parts = asset_parts(include_bytes!("../../assets/logan/model.obj"));
    let mesh = &parts[0].mesh;
    // Source bumper cross-sections, behind/in front of the wheel openings.
    // Check both absence below the hem and retained outer skin above it.
    for (y, hem_z, width) in [
        (-1.74, -0.252, 0.900),
        (-1.80, -0.250, 0.890),
        (-1.90, -0.247, 0.860),
        (1.89, -0.309, 0.923),
        (1.94, -0.308, 0.911),
    ] {
        for side in [-1., 1.] {
            let direction = Vec3::new(-side, 0., 0.);
            assert!(
                mesh_ray(mesh, Vec3::new(side * 2., y, hem_z - 0.012), direction).is_none(),
                "hanging bumper tab at {side},{y}"
            );
            let t = mesh_ray(mesh, Vec3::new(side * 2., y, hem_z + 0.015), direction)
                .expect("bumper corner was removed instead of fitted");
            assert!(
                (2. - t - width).abs() < 0.04,
                "bumper hem left the source skin at {side},{y}"
            );
        }
    }
}

fn mesh_ray(mesh: &tobj::Mesh, origin: glam::Vec3, direction: glam::Vec3) -> Option<f32> {
    use glam::Vec3;
    mesh.indices
        .as_chunks::<3>()
        .0
        .iter()
        .filter_map(|tri| {
            let [a, b, c] =
                tri.map(|i| Vec3::from_slice(&mesh.positions[3 * i as usize..3 * i as usize + 3]));
            let e = b - a;
            let f = c - a;
            let h = direction.cross(f);
            let det = e.dot(h);
            if det.abs() < 1e-8 {
                return None;
            }
            let s = origin - a;
            let u = s.dot(h) / det;
            let q = s.cross(e);
            let v = direction.dot(q) / det;
            let t = f.dot(q) / det;
            (u >= -1e-5 && v >= -1e-5 && u + v <= 1.00001 && t >= 0.0).then_some(t)
        })
        .min_by(f32::total_cmp)
}

#[test]
fn all_wheel_arches_follow_the_source_apertures() {
    use glam::Vec3;
    // Silhouette measurements from the edited sources, not fitted circles.
    // Front/rear shoulders and crowns constrain all eight wheel openings.
    for (name, bytes, width, samples) in [
        (
            "logan",
            include_bytes!("../../assets/logan/model.obj").as_slice(),
            0.90,
            [
                (-1.60, 0.028),
                (-1.30, 0.162),
                (-1.0, -0.038),
                (1.15, 0.022),
                (1.45, 0.198),
                (1.75, 0.062),
            ],
        ),
        (
            "l200",
            include_bytes!("../../assets/l200/model.obj").as_slice(),
            0.99,
            [
                (-1.80, -0.381),
                (-1.36, -0.126),
                (-0.90, -0.423),
                (1.55, -0.388),
                (1.97, -0.164),
                (2.40, -0.446),
            ],
        ),
    ] {
        let parts = asset_parts(bytes);
        for (y, z) in samples {
            for side in [-1.0, 1.0] {
                for (offset, expected_hit) in [(-0.007, false), (0.007, true)] {
                    let origin = Vec3::new(side * 3., y, z + offset);
                    let hit = parts
                        .iter()
                        .filter_map(|p| mesh_ray(&p.mesh, origin, Vec3::new(-side, 0., 0.)))
                        .any(|distance| distance < 3.0 - width);
                    assert_eq!(
                        hit, expected_hit,
                        "{name} side {side} arch at {y},{z} offset {offset}"
                    );
                }
            }
        }
    }
}

#[test]
fn l200_handles_stay_on_the_door_sheet() {
    use glam::Vec3;
    let parts = asset_parts(include_bytes!("../../assets/l200/model.obj"));
    // Measurements of the surrounding door paint, excluding the handle and cup.
    for (y, z, width) in [
        (0.4, 0.035, 1.120),
        (-0.64, 0.09, 1.111),
        (0.4, -0.025, 1.136),
        (-0.64, 0.04, 1.128),
    ] {
        for sign in [-1.0, 1.0] {
            let origin = Vec3::new(sign * 2., y, z);
            let distance = parts
                .iter()
                .filter_map(|p| mesh_ray(&p.mesh, origin, Vec3::new(-sign, 0., 0.)))
                .min_by(f32::total_cmp)
                .unwrap();
            assert!(
                (2.0 - distance - width).abs() < 0.008,
                "handle at {y},{z}: panel width {}",
                2.0 - distance
            );
        }
    }
}

#[test]
fn l200_has_black_floor_and_closed_inner_wheel_houses() {
    use glam::Vec3;
    use wgame::image::{Image, ImageBase, ImageRead};
    let parts = asset_parts(include_bytes!("../../assets/l200/details.obj"));
    assert!(
        !parts
            .iter()
            .any(|p| p.name == "rear_axle" || p.name == "source_underbody")
    );
    assert!(
        !parts
            .iter()
            .any(|p| p.name == "cab_bed_join" || p.name == "source_rear")
    );
    let floor = &parts
        .iter()
        .find(|p| p.name == "opaque_underbody")
        .unwrap()
        .mesh;
    assert!(
        floor
            .positions
            .as_chunks::<3>()
            .0
            .iter()
            .all(|p| { p[1] >= -2.1 || p[0].abs() < 1.076 }),
        "underbody protrudes beyond the narrower rear bed corners"
    );
    let image = Image::decode_auto(include_bytes!("../../assets/l200/details.png")).unwrap();
    for uv in floor.texcoords.as_chunks::<2>().0 {
        let size = image.size();
        let x = (uv[0] * size.width as f32) as u32;
        let y = (uv[1] * size.height as f32) as u32;
        let c = image.data()[(y * size.width + x) as usize];
        assert!(
            c.r.to_f32() == 0.0
                && c.g.to_f32() == 0.0
                && c.b.to_f32() == 0.0
                && c.a.to_f32() == 1.0
        );
    }
    for x in [-0.7, 0., 0.7] {
        for y in [-2.5, -2., -1.36, -0.96, -0.5, 0., 0.5, 1., 1.5, 1.97, 2.4] {
            let t = mesh_ray(floor, Vec3::new(x, y, -2.), Vec3::Z).expect("hole in floor");
            assert!(
                (1.46..1.52).contains(&t),
                "floor is not recessed at {x},{y}: {t}"
            );
        }
    }
    for y in [-1.36, 1.97] {
        for sign in [-1.0, 1.0] {
            let t = mesh_ray(
                floor,
                Vec3::new(sign * 2., y, -0.3),
                Vec3::new(-sign, 0., 0.),
            )
            .expect("open wheel house");
            assert!(
                (1.2..1.35).contains(&t),
                "wheel house does not close behind the wheel: {t}"
            );
        }
    }
}

#[test]
fn l200_bed_uv_panels_do_not_overlap() {
    use glam::Vec2;
    let parts = asset_parts(include_bytes!("../../assets/l200/model.obj"));
    let bed = &parts.iter().find(|p| p.name == "l200_bed").unwrap().mesh;
    const SIZE: usize = 512;
    let mut occupied = vec![false; SIZE * SIZE];
    for tri in bed.indices.as_chunks::<3>().0 {
        let [a, b, c] = tri.map(|i| {
            Vec2::from_slice(&bed.texcoords[2 * i as usize..2 * i as usize + 2]) * SIZE as f32
        });
        let det = (b - a).perp_dot(c - a);
        if det.abs() < 1e-6 {
            continue;
        }
        let lo = a.min(b).min(c).floor().max(Vec2::ZERO).as_uvec2();
        let hi = a
            .max(b)
            .max(c)
            .ceil()
            .min(Vec2::splat(SIZE as f32))
            .as_uvec2();
        for y in lo.y..hi.y {
            for x in lo.x..hi.x {
                let p = Vec2::new(x as f32 + 0.5, y as f32 + 0.5) - a;
                let u = p.perp_dot(c - a) / det;
                let v = (b - a).perp_dot(p) / det;
                // Exclude shared edges so rounding cannot count a legitimate seam twice.
                if u > 1e-4 && v > 1e-4 && u + v < 0.9999 {
                    let pixel = y as usize * SIZE + x as usize;
                    assert!(!occupied[pixel], "bed UV panels overlap at {x},{y}");
                    occupied[pixel] = true;
                }
            }
        }
    }
}

#[test]
fn l200_liner_walls_keep_their_inward_surface_normals() {
    use glam::Vec3;
    use wgame::image::{Image, ImageBase, ImageRead};
    let parts = asset_parts(include_bytes!("../../assets/l200/model.obj"));
    let mesh = &parts.iter().find(|p| p.name == "l200_bed").unwrap().mesh;
    let image = Image::decode_auto(include_bytes!("../../assets/l200/normal.png")).unwrap();
    let size = image.size();
    let mut checked = 0;
    for tri in mesh.indices.as_chunks::<3>().0 {
        let p = tri.map(|i| Vec3::from_slice(&mesh.positions[3 * i as usize..3 * i as usize + 3]));
        let center = (p[0] + p[1] + p[2]) / 3.;
        let cross = (p[1] - p[0]).cross(p[2] - p[0]);
        let Some(geometric) = cross.try_normalize() else {
            continue;
        };
        let front = (center.y + 1.04).abs() < 1e-5 && geometric.y < -0.999 && center.z < 0.25;
        let side = (center.x.abs() - 0.98).abs() < 1e-5 && center.x * geometric.x < -0.97;
        if !front && !side {
            continue;
        }
        let uv = tri
            .iter()
            .map(|&i| glam::Vec2::from_slice(&mesh.texcoords[2 * i as usize..2 * i as usize + 2]))
            .sum::<glam::Vec2>()
            / 3.;
        let x = (uv.x * size.width as f32) as u32;
        let y = (uv.y * size.height as f32) as u32;
        let c = image.data()[(y * size.width + x) as usize];
        let mapped = Vec3::new(c.r.to_f32(), c.g.to_f32(), c.b.to_f32()) * 2. - Vec3::ONE;
        assert!(
            mapped.dot(geometric) > 0.99,
            "liner normals blend into the opposite skin at {center:?}"
        );
        checked += 1;
    }
    assert!(checked > 20);
}

#[test]
fn l200_lower_bed_corners_bake_the_outer_paint_and_outward_normals() {
    use glam::{Vec2, Vec3};
    use wgame::image::{Image, ImageBase, ImageRead};
    let parts = asset_parts(include_bytes!("../../assets/l200/model.obj"));
    let bed = &parts.iter().find(|p| p.name == "l200_bed").unwrap().mesh;
    let color = Image::decode_auto(include_bytes!("../../assets/l200/color.png")).unwrap();
    let normal = Image::decode_auto(include_bytes!("../../assets/l200/normal.png")).unwrap();
    for side in [-1., 1.] {
        for (y, z) in [(-0.68346, -0.54789), (-0.65, -0.65)] {
            let mut nearest = None;
            for tri in bed.indices.as_chunks::<3>().0 {
                let p = tri
                    .map(|i| Vec3::from_slice(&bed.positions[3 * i as usize..3 * i as usize + 3]));
                let [a, b, c] = p.map(|v| Vec2::new(v.y, v.z));
                let det = (b - a).perp_dot(c - a);
                if det.abs() < 1e-8 {
                    continue;
                }
                let q = Vec2::new(y, z) - a;
                let u = q.perp_dot(c - a) / det;
                let v = (b - a).perp_dot(q) / det;
                if u < 0. || v < 0. || u + v > 1. {
                    continue;
                }
                let weights = [1. - u - v, u, v];
                let x = (0..3).map(|i| weights[i] * p[i].x * side).sum::<f32>();
                let uv = (0..3)
                    .map(|i| {
                        let j = tri[i] as usize * 2;
                        Vec2::from_slice(&bed.texcoords[j..j + 2]) * weights[i]
                    })
                    .sum::<Vec2>();
                if nearest.is_none_or(|(width, _)| x > width) {
                    nearest = Some((x, uv));
                }
            }
            let (width, uv) = nearest.expect("missing lower bed corner");
            assert!(width > 1.);
            let sample = |image: &Image<_>| {
                let size = image.size();
                image.data()[((uv.y * size.height as f32) as u32 * size.width
                    + (uv.x * size.width as f32) as u32) as usize]
            };
            let c = sample(&color);
            assert!(
                c.r.to_f32() > 0.6 && c.g.to_f32() > 0.6 && c.b.to_f32() > 0.6,
                "paint projection missed the outer skin at {side},{y},{z}"
            );
            let n = sample(&normal);
            assert!(
                side * (2. * n.r.to_f32() - 1.) > 0.6,
                "inward normal at lower bed corner {side},{y},{z}"
            );
        }
    }
}

#[test]
fn l200_bed_arch_shoulders_do_not_fold_back_into_the_side_panel() {
    use glam::Vec3;
    let parts = asset_parts(include_bytes!("../../assets/l200/model.obj"));
    let bed = &parts.iter().find(|p| p.name == "l200_bed").unwrap().mesh;
    let mut checked = [0, 0];
    for indices in bed.indices.as_chunks::<3>().0 {
        let p = indices.map(|i| {
            let i = i as usize * 3;
            Vec3::from_slice(&bed.positions[i..i + 3])
        });
        for (side_index, side) in [-1., 1.].into_iter().enumerate() {
            // Exclude the rim, front closing wall and interior; inspect the
            // outward sheet around the steep front shoulder on both sides.
            if p.iter()
                .all(|v| side * v.x > 1.035 && v.y > -1.15 && v.y < -0.60 && v.z < 0.13)
            {
                let normal = (p[1] - p[0]).cross(p[2] - p[0]);
                assert!(side * normal.x > 0., "folded cargo side triangle: {p:?}");
                checked[side_index] += 1;
            }
        }
    }
    assert!(checked.into_iter().all(|n| n > 10));
}

#[test]
fn l200_bed_uses_broad_panels_without_handle_geometry() {
    use glam::Vec3;
    let parts = asset_parts(include_bytes!("../../assets/l200/model.obj"));
    let bed = &parts.iter().find(|p| p.name == "l200_bed").unwrap().mesh;
    assert!(
        bed.indices.len() / 3 < 2000,
        "cargo mesh exceeds its detail budget"
    );
    // The handle and its cup belong in the bake: the tailgate remains a sheet.
    for z in [-0.10, 0., 0.10, 0.15] {
        let distances: Vec<_> = [-0.4, -0.2, 0., 0.2, 0.4]
            .into_iter()
            .map(|x| mesh_ray(bed, Vec3::new(x, -3., z), Vec3::Y).unwrap())
            .collect();
        for d in &distances {
            assert!(
                (d - distances[0]).abs() < 0.001,
                "handle deforms tailgate at {z}"
            );
        }
    }
    for x in [-0.65, 0., 0.65] {
        for y in [-2.4, -2.0, -1.6, -1.2] {
            let t = mesh_ray(bed, Vec3::new(x, y, 1.), -Vec3::Z).expect("hole in cargo floor");
            assert!(
                (t - 1.276).abs() < 0.001,
                "load floor is not flat at {x},{y}"
            );
        }
    }
    let details = asset_parts(include_bytes!("../../assets/l200/details.obj"));
    let lining = &details
        .iter()
        .find(|p| p.name == "opaque_underbody")
        .unwrap()
        .mesh;
    for x in [-0.95, -0.84, -0.8, 0., 0.8, 0.84, 0.95] {
        for y in [-1.8, -1.6, -1.4, -1.2] {
            let origin = Vec3::new(x, y, 1.);
            let cargo = mesh_ray(bed, origin, -Vec3::Z).unwrap();
            let underside = mesh_ray(lining, origin, -Vec3::Z).unwrap();
            assert!(
                cargo + 0.005 < underside,
                "black wheel house intrudes into cargo bed at {x},{y}"
            );
        }
    }
}

#[test]
fn l200_cab_and_bed_are_independent_shells_with_separate_atlas_regions() {
    use glam::Vec3;
    let parts = asset_parts(include_bytes!("../../assets/l200/model.obj"));
    assert_eq!(parts.len(), 2);
    let cab = &parts
        .iter()
        .find(|p| p.name == "l200_cab")
        .expect("cab missing")
        .mesh;
    let bed = &parts
        .iter()
        .find(|p| p.name == "l200_bed")
        .expect("bed missing")
        .mesh;
    assert!(
        cab.positions
            .as_chunks::<3>()
            .0
            .iter()
            .all(|p| p[1] > -1.04)
    );
    assert!(
        bed.positions
            .as_chunks::<3>()
            .0
            .iter()
            .all(|p| p[1] < -0.58)
    );
    assert!(
        cab.texcoords
            .as_chunks::<2>()
            .0
            .iter()
            .all(|uv| uv[0] < 0.65)
    );
    assert!(
        bed.texcoords
            .as_chunks::<2>()
            .0
            .iter()
            .all(|uv| uv[0] > 0.68)
    );
    // Rear glazing belongs to the cab. The bed must not acquire a sloping
    // connector that covers it or bridges the cargo opening.
    let origin = Vec3::new(0., -1.2, 0.45);
    let rear = mesh_ray(cab, origin, Vec3::Y).expect("cab rear is open");
    assert!(
        (0.15..0.30).contains(&rear),
        "cab rear projected into the interior: {rear}"
    );
    assert!(mesh_ray(bed, origin, Vec3::Y).is_none());
    // The reconstructed rear pillar and door sheet share one boundary.
    // Different tessellations must not leave slits within the cab itself.
    for z in [-0.6_f32, -0.3, 0., 0.15, 0.3, 0.5, 0.65] {
        for offset in [-0.003, 0., 0.003] {
            let y = -0.78 - 0.5 * z.min(0.16) + offset;
            for side in [-1., 1.] {
                let distance = mesh_ray(cab, Vec3::new(side * 2., y, z), Vec3::new(-side, 0., 0.))
                    .expect("slit between cab pillar and door sheet");
                assert!(distance < 1.25, "ray passed through the cab side");
            }
        }
    }
}

#[test]
fn paint_masks_match_assets_and_keep_dark_trim_unpainted() {
    use super::*;
    for (base, paint) in [
        (
            &include_bytes!("../../assets/logan/color.png")[..],
            &include_bytes!("../../assets/logan/paint.png")[..],
        ),
        (
            &include_bytes!("../../assets/l200/color.png")[..],
            &include_bytes!("../../assets/l200/paint.png")[..],
        ),
        (
            &include_bytes!("../../assets/logan/details.png")[..],
            &include_bytes!("../../assets/logan/details-paint.png")[..],
        ),
        (
            &include_bytes!("../../assets/l200/details.png")[..],
            &include_bytes!("../../assets/l200/details-paint.png")[..],
        ),
    ] {
        let base = Image::decode_auto(base).unwrap();
        let mask = Image::decode_auto(paint).unwrap();
        assert_eq!(base.size(), mask.size());
        let mut painted = 0;
        for (pos, pixel) in base.pixels() {
            let coverage = f32::from(mask.get(pos).r);
            if coverage > 0.99 {
                painted += 1;
            }
            if pixel.r.to_f32().max(pixel.g.to_f32()).max(pixel.b.to_f32()) < 0.15 {
                assert_eq!(coverage, 0.0, "dark glass/rubber must retain its albedo");
            }
        }
        assert!(painted >= 256, "body and detail maps need a painted region");
    }
}

#[test]
#[ignore = "requires a GPU adapter; run with --ignored"]
fn vehicle_paint_changes_body_but_preserves_trim_and_wheels() {
    use super::*;
    use crate::camera::Orbit;
    let gfx = graphics();
    let lib = Library::new(&gfx);
    let terrain = Terrain::from_height_map(|_| 0.0, 64.0, 24);
    let assets = Assets::new(&lib, &terrain).unwrap();
    let mut target = Offscreen::new(&gfx, (960, 600));
    for model in 0..2 {
        let car = crate::spawn(model).unwrap();
        let mut orbit = Orbit::default();
        orbit.rotate(glam::Vec2::new(620.0, -50.0));
        orbit.zoom(2.0);
        let (matrix, eye) = orbit.view(car.pos(), &terrain, 1.6);
        assets.update_lighting(eye).unwrap();
        let camera = wgame::gfx::Camera::new(&gfx, matrix);
        let render = |target: &mut Offscreen, paint: [u8; 3]| {
            let mut scene = Scene::default();
            assets.draw_vehicle(
                &car,
                model,
                crate::appearance::linear_color(paint),
                &mut scene,
            );
            target.clear(Vec4::new(0.18, 0.18, 0.18, 1.0));
            target.render(&camera, &scene.bake());
            pixels(target)
        };
        let red = render(&mut target, [210, 35, 20]);
        let blue = render(&mut target, [25, 65, 210]);
        let mut changed = 0;
        let mut preserved = 0;
        for (a, b) in red
            .as_chunks::<4>()
            .0
            .iter()
            .zip(blue.as_chunks::<4>().0.iter())
        {
            if a[0].abs_diff(b[0]) > 30 && a[2].abs_diff(b[2]) > 30 {
                changed += 1;
            }
            if a == b && a[0] < 35 && a[1] < 35 && a[2] < 35 {
                preserved += 1;
            }
        }
        assert!(changed > 5000, "painted body should change: {changed}");
        assert!(
            preserved > 1000,
            "glass, tires and trim should stay dark: {preserved}"
        );
        let name = if model == 0 { "logan" } else { "l200" };
        save_image(&format!("{name}-paint-red"), target.size(), &red);
        save_image(&format!("{name}-paint-blue"), target.size(), &blue);
    }
}
