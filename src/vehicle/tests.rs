use super::*;
fn config() -> VehicleConfig {
    serde_json::from_slice(include_bytes!("../../assets/logan/config.json")).unwrap()
}
fn flat() -> Terrain {
    Terrain::from_height_map(|_| 0.0, 200.0, 8)
}
fn car() -> Vehicle {
    Vehicle::new(config(), Vec3::new(0.0, 0.0, 3.0), Quat::IDENTITY)
}
fn advance(car: &mut Vehicle, terrain: &Terrain, seconds: f32, dt: f32) {
    for _ in 0..(seconds / dt).round() as u32 {
        car.step(terrain, dt);
        assert!(car.pos().is_finite() && car.velocity().is_finite());
    }
}
#[test]
fn airborne_free_fall_and_zero_step() {
    let terrain = Terrain::from_height_map(|_| -100.0, 100.0, 2);
    let mut car = car();
    car.step(&terrain, 0.0);
    assert_eq!(car.pos().z, 3.0);
    advance(&mut car, &terrain, 0.5, 1.0 / 240.0);
    assert!((car.pos().z - (3.0 - 0.5 * 9.8 * 0.5 * 0.5)).abs() < 0.001);
}
#[test]
fn suspension_settles_and_braking_stops_the_car() {
    let terrain = flat();
    let mut car = car();
    advance(&mut car, &terrain, 8.0, 1.0 / 240.0);
    assert!(car.pos().z > 0.0 && car.pos().z < 1.0, "{:?}", car.pos());
    assert!(car.velocity().length() < 0.05, "{:?}", car.velocity());
    car.accelerate(1.0);
    advance(&mut car, &terrain, 4.0, 1.0 / 240.0);
    assert!(car.pos().y > 3.0, "{:?}", car.pos());
    car.brake();
    advance(&mut car, &terrain, 4.0, 1.0 / 240.0);
    assert!(car.velocity().length() < 0.1, "{:?}", car.velocity());
}
#[test]
fn fixed_step_refinement_agrees_for_resting_suspension() {
    let terrain = flat();
    let mut a = car();
    let mut b = car();
    advance(&mut a, &terrain, 6.0, 1.0 / 240.0);
    advance(&mut b, &terrain, 6.0, 1.0 / 480.0);
    assert!(
        (a.pos() - b.pos()).length() < 0.02,
        "{:?} {:?}",
        a.pos(),
        b.pos()
    );
}

#[test]
fn both_models_reverse_and_turn_on_terrain() {
    let terrain = flat();
    for json in [
        &include_bytes!("../../assets/logan/config.json")[..],
        &include_bytes!("../../assets/l200/config.json")[..],
    ] {
        let config = serde_json::from_slice(json).unwrap();
        let mut car = Vehicle::new(config, Vec3::new(0.0, 0.0, 3.0), Quat::IDENTITY);
        advance(&mut car, &terrain, 6.0, 1.0 / 240.0);
        car.accelerate(-0.5);
        advance(&mut car, &terrain, 3.0, 1.0 / 240.0);
        assert!(car.pos().y < -1.0, "{:?}", car.pos());
        car.accelerate(1.0);
        car.steer(std::f32::consts::FRAC_PI_6);
        advance(&mut car, &terrain, 5.0, 1.0 / 240.0);
        assert!(car.pos().x.abs() > 1.0, "{:?}", car.pos());
        let up = car.transform().transform_vector3(Vec3::Z);
        assert!(up.z > 0.8, "{up:?}");
        assert!(car.wheel_transforms().iter().all(|m| m.is_finite()));
    }
}

#[test]
fn unloaded_suspension_never_pulls_toward_ground() {
    let mut wheel = Wheel::new(config().wheel_common, config().wheels[0].clone());
    wheel.dev = 0.01;
    assert_eq!(wheel.normal_reaction(Vec3::Z, Vec3::Z * 100.0), Vec3::ZERO);
}

#[test]
fn sloped_ground_supports_a_braked_vehicle() {
    let terrain = Terrain::from_height_map(|p| 0.1 * p.x, 100.0, 8);
    let mut car = car();
    car.brake();
    advance(&mut car, &terrain, 8.0, 1.0 / 240.0);
    assert!(car.velocity().length() < 0.1, "{:?}", car.velocity());
    let up = car.transform().transform_vector3(Vec3::Z);
    assert!(
        up.dot(Vec3::new(-0.1, 0.0, 1.0).normalize()) > 0.99,
        "{up:?}"
    );
}
