use super::{asset_parts, mesh_ray};

#[test]
fn logan_trunk_lid_keeps_its_top_contour_up_to_the_rear_fold() {
    use glam::Vec3;
    let parts = asset_parts(include_bytes!("../../../assets/logan/model.obj"));
    // Heights measured on the edited high-poly lid. A wide diagonal spanning
    // this fold used to pull the low-poly surface down by several centimeters.
    for (x, y, expected) in [
        (0.0, -2.00, 0.6241),
        (0.0, -2.06, 0.6136),
        (0.0, -2.09, 0.6084),
        (0.4, -2.00, 0.6132),
        (0.4, -2.06, 0.6034),
        (0.4, -2.09, 0.5981),
        (0.65, -2.00, 0.5964),
        (0.65, -2.03, 0.5908),
        (0.75, -2.00, 0.5935),
        (0.8, -2.00, 0.5838),
    ] {
        for side in [-1.0, 1.0] {
            let distance = parts
                .iter()
                .filter_map(|part| mesh_ray(&part.mesh, Vec3::new(side * x, y, 2.0), -Vec3::Z))
                .min_by(f32::total_cmp)
                .expect("missing trunk surface");
            assert!(
                (2.0 - distance - expected).abs() < 0.012,
                "trunk lid droops at {side} * {x}, {y}: height {}",
                2.0 - distance
            );
        }
    }
}

#[test]
fn l200_paint_includes_shaded_panel_edges_but_not_seals() {
    use wgame::image::{Image, ImageReadExt};
    let mask = Image::decode_auto(include_bytes!("../../../assets/l200/paint.png")).unwrap();
    // Authored silver bevels around hood, doors and tailgate have different
    // brightness from the broad panels, but belong to the same paint material.
    for (x, y) in [
        (238, 700),
        (241, 700),
        (201, 2300),
        (211, 2300),
        (118, 1700),
        (3423, 1800),
        (3424, 1800),
    ] {
        assert_eq!(
            f32::from(mask.get((x, y).into()).r),
            1.0,
            "unpainted panel bevel at {x}, {y}"
        );
    }
    for (x, y) in [(234, 700), (235, 700), (236, 700)] {
        assert_eq!(
            f32::from(mask.get((x, y).into()).r),
            0.0,
            "neutral seal must stay unpainted at {x}, {y}"
        );
    }
}

#[test]
fn logan_trunk_bake_keeps_red_lamps_visible_through_the_source_covers() {
    use wgame::image::{Image, ImageReadExt};
    let color = Image::decode_auto(include_bytes!("../../../assets/logan/color.png")).unwrap();
    // Rear-chart lamp regions: the clear outer lens must not replace the red
    // source beneath it with its own dark base color during reprojection.
    for (left, right) in [(3420, 3630), (3820, 4035)] {
        let mut red = 0;
        for y in 2900..3150 {
            for x in left..right {
                let p = color.get((x, y).into());
                if f32::from(p.r) > 0.7 && f32::from(p.g) < 0.2 && f32::from(p.b) < 0.2 {
                    red += 1;
                }
            }
        }
        assert!(
            red > 1500,
            "rear lamp lost behind opaque cover: {red} red texels"
        );
    }
}
