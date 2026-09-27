use super::{graphics, pixels, save_image};

#[test]
#[ignore = "requires a GPU adapter; run with --ignored"]
fn wheel_has_an_open_barrel_when_viewed_from_inside() {
    use crate::render::*;
    use glam::{Affine3A, Mat4};
    use wgame::gfx::{Camera, Offscreen};
    let gfx = graphics();
    let lib = Library::new(&gfx);
    let lighting =
        Lighting::new(lib.shapes(), lib.texturing(), LightParameters::default()).unwrap();
    let color = texture(&lib, include_bytes!("../../../assets/wheel/color.png")).unwrap();
    let normal = texture(&lib, include_bytes!("../../../assets/wheel/normal.png")).unwrap();
    let material = lighting
        .material(
            Some(&normal),
            MaterialSettings {
                normal_y: NormalY::Positive,
                specular: 0.08,
                ..Default::default()
            },
        )
        .unwrap();
    let rim_material = lighting
        .material(None, MaterialSettings::default())
        .unwrap();
    let transform = Affine3A::from_scale(Vec3::new(1.0, 1.0, 0.65));
    let mut scene = Scene::default();
    scene.add(
        &wheel_mesh(&lib, &color)
            .with_material(&material)
            .transform(transform),
    );
    scene.add(
        &wheel::barrel(&lib)
            .with_material(&rim_material)
            .transform(transform),
    );
    let face = Image::decode_auto(include_bytes!("../../../assets/wheel/color.png")).unwrap();
    for (name, eye) in [
        ("outer", Vec3::new(1.7, 0.8, 3.4)),
        ("inner", Vec3::new(-1.7, 0.8, -3.4)),
    ] {
        lighting
            .update(LightParameters {
                eye,
                direction: Vec3::new(0.3, 1.0, 0.4),
                ..Default::default()
            })
            .unwrap();
        let matrix: Mat4 = glam::camera::rh::proj::directx::perspective(0.65, 1.0, 0.1, 20.0)
            * glam::camera::rh::view::look_at_mat4(eye, Vec3::ZERO, Vec3::Y);
        let camera = Camera::new(&gfx, matrix);
        let mut target = Offscreen::new(&gfx, (700, 700));
        target.clear(Vec4::new(0.08, 0.17, 0.25, 1.0));
        target.render(&camera, &scene.bake());
        let rendered = pixels(&mut target);
        save_image(&format!("wheel-{name}-face"), target.size(), &rendered);
        if name == "inner" {
            // Rays through spoke holes on the sole outer disk must see the
            // background, including where an extra inner disk would be opaque.
            let inverse = matrix.inverse();
            let mut checked = 0;
            let opaque = |p: Vec3, alpha: f32| {
                let x = ((p.x + 1.0) * 128.0) as u32;
                let y = ((p.y + 1.0) * 128.0) as u32;
                (y - 1..=y + 1).all(|yy| {
                    (x - 1..=x + 1)
                        .all(|xx| (f32::from(face.get((xx, yy).into()).a) - alpha).abs() < 0.01)
                })
            };
            for y in (0..700).step_by(4) {
                for x in (0..700).step_by(4) {
                    let point = inverse
                        * Vec4::new(
                            (x as f32 + 0.5) / 350.0 - 1.0,
                            1.0 - (y as f32 + 0.5) / 350.0,
                            1.0,
                            1.0,
                        );
                    let ray = point.truncate() / point.w - eye;
                    let inner = eye + ray * ((-0.325 - eye.z) / ray.z);
                    let outer = eye + ray * ((0.325 - eye.z) / ray.z);
                    if inner.truncate().length() < 0.65
                        && outer.truncate().length() < 0.65
                        && opaque(inner, 1.0)
                        && opaque(outer, 0.0)
                    {
                        let pixel = &rendered[(y * 700 + x) * 4..][..3];
                        assert!(
                            pixel
                                .iter()
                                .zip([20, 43, 64])
                                .all(|(a, b)| a.abs_diff(b) <= 1),
                            "extra inner spoke disk blocks outer hole at {x}, {y}: {pixel:?}"
                        );
                        checked += 1;
                    }
                }
            }
            assert!(
                checked > 25,
                "not enough clear rays through the rim: {checked}"
            );
        }
    }
}
