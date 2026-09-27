use super::*;
fn config() -> VehicleConfig {
    serde_json::from_slice(include_bytes!("../../assets/logan/config.json")).unwrap()
}
fn flat() -> Terrain {
    Terrain::from_height_map(|_| 0.0, 2000.0, 2)
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
    car.accelerate(1.0);
    advance(&mut car, &terrain, 0.5, 1.0 / 240.0);
    assert!((car.pos().z - (3.0 - 0.5 * 9.8 * 0.5 * 0.5)).abs() < 0.001);
    assert_eq!(car.velocity().y, 0.0);
    assert!(car.wheels.iter().all(|wheel| wheel.rot.angle() == 0.0));
}

fn configs() -> [VehicleConfig; 2] {
    [
        &include_bytes!("../../assets/logan/config.json")[..],
        &include_bytes!("../../assets/l200/config.json")[..],
    ]
    .map(|json| serde_json::from_slice(json).unwrap())
}

fn settled(config: VehicleConfig, terrain: &Terrain, dt: f32) -> Vehicle {
    let mut car = Vehicle::new(config, Vec3::new(0.0, 0.0, 3.0), Quat::IDENTITY);
    car.brake();
    advance(&mut car, terrain, 6.0, dt);
    car.reset_controls();
    car
}

#[test]
fn excess_launch_torque_does_not_reduce_traction_or_force_wheel_overspeed() {
    let terrain = flat();
    let dt = 1.0 / 240.0;
    for config in configs() {
        for steering in [0.0, 0.2] {
            let mut excessive = config.clone();
            excessive.max_torque *= 100.0;
            // Keep power (and hence calibrated aerodynamic drag) unchanged.
            let mut normal = settled(config.clone(), &terrain, dt);
            let mut high = settled(excessive, &terrain, dt);
            for car in [&mut normal, &mut high] {
                car.accelerate(1.0);
                car.steer(steering);
                car.step(&terrain, dt);
                assert!(car.wheels.iter().all(|wheel| wheel.visible_asp.abs() < 0.2));
                advance(car, &terrain, 2.0, dt);
            }
            let speed = normal.velocity().length();
            assert!(
                speed > 2.0,
                "mass={}, steering={steering}, launch speed={speed}",
                config.mass
            );
            assert!(
                high.velocity().length() >= speed * 0.99,
                "mass={}, steering={steering}, normal={:?}, high={:?}",
                config.mass,
                normal.velocity(),
                high.velocity()
            );
            assert!(normal.rot.transform(Vec3::Z).z > 0.9);
        }
    }
}

#[test]
fn acceleration_falls_with_speed_and_partial_throttle_still_exceeds_old_speed_limit() {
    let terrain = flat();
    let dt = 1.0 / 240.0;
    for config in configs() {
        let acceleration = [10.0, 45.0].map(|speed| {
            let mut car = settled(config.clone(), &terrain, dt);
            *car.vel = Vec3::Y * speed;
            car.accelerate(1.0);
            advance(&mut car, &terrain, 1.0, dt);
            car.velocity().y - speed
        });
        assert!(acceleration[1] > 0.0, "{acceleration:?}");
        assert!(acceleration[1] < 0.8 * acceleration[0], "{acceleration:?}");

        let mut car = settled(config, &terrain, dt);
        car.accelerate(0.25);
        advance(&mut car, &terrain, 10.0, dt);
        assert!(car.velocity().y > 6.0, "{:?}", car.velocity());
    }
}

#[test]
fn coasting_loses_speed_and_direction_changes_brake_before_reversing() {
    let terrain = flat();
    let dt = 1.0 / 240.0;
    for config in configs() {
        let reverse_speed = config.max_reverse_speed;
        let mut car = settled(config, &terrain, dt);
        *car.vel = Vec3::Y * 8.0;
        car.accelerate(0.0);
        advance(&mut car, &terrain, 2.0, dt);
        assert!(car.velocity().y > 0.0 && car.velocity().y < 7.9);
        car.accelerate(-1.0);
        assert_eq!(car.pedals(), (0.0, true));
        let before = car.velocity().y;
        car.step(&terrain, dt);
        assert!(car.velocity().y > 0.0 && car.velocity().y < before);
        advance(&mut car, &terrain, 8.0, dt);
        assert!(
            car.velocity().y < -0.8 * reverse_speed,
            "{:?}",
            car.velocity()
        );
        assert!(
            car.velocity().y >= -reverse_speed - 0.05,
            "{:?}",
            car.velocity()
        );
        car.brake();
        advance(&mut car, &terrain, 3.0, dt);
        assert!(car.velocity().length() < 0.05, "{:?}", car.velocity());
    }
}

#[test]
fn more_driven_wheels_share_engine_power_instead_of_multiplying_it() {
    let terrain = flat();
    let dt = 1.0 / 240.0;
    let mut front_drive = config();
    front_drive.max_power = 20_000.0;
    let mut all_drive = front_drive.clone();
    for wheel in &mut all_drive.wheels {
        wheel.drive = true;
    }
    let [front, all] = [front_drive, all_drive].map(|config| {
        let mut car = settled(config, &terrain, dt);
        *car.vel = Vec3::Y * 30.0;
        car.accelerate(1.0);
        advance(&mut car, &terrain, 1.0, dt);
        car.velocity().y - 30.0
    });
    assert!(front > 0.1);
    assert!((front - all).abs() < 0.01, "front={front}, all={all}");
}

#[test]
fn level_ground_top_speed_balances_resistance() {
    let terrain = flat();
    let dt = 1.0 / 240.0;
    for config in configs() {
        let speed = config.max_speed;
        let mut car = settled(config, &terrain, dt);
        *car.vel = Vec3::Y * speed;
        car.accelerate(1.0);
        advance(&mut car, &terrain, 8.0, dt);
        assert!(
            (car.velocity().y - speed).abs() < 0.5,
            "{:?}",
            car.velocity()
        );
    }
}

#[test]
fn hill_starts_use_grip_without_spinning_the_wheels() {
    let terrain = Terrain::from_height_map(|p| 0.08 * p.y, 200.0, 2);
    let dt = 1.0 / 240.0;
    for config in configs() {
        let mut car = settled(config, &terrain, dt);
        let start = car.pos();
        car.accelerate(1.0);
        advance(&mut car, &terrain, 3.0, dt);
        assert!(car.pos().y > start.y + 1.0, "{:?} {start:?}", car.pos());
        assert!(car.velocity().y > 0.5, "{:?}", car.velocity());
    }
}

#[test]
fn driving_cornering_and_braking_agree_when_timestep_is_halved() {
    let terrain = flat();
    for config in configs() {
        let [a, b] = [1.0 / 240.0, 1.0 / 480.0].map(|dt| {
            let mut car = settled(config.clone(), &terrain, dt);
            car.accelerate(1.0);
            advance(&mut car, &terrain, 4.0, dt);
            car.steer(0.12);
            advance(&mut car, &terrain, 2.0, dt);
            car.reset_controls();
            advance(&mut car, &terrain, 2.0, dt);
            car.brake();
            advance(&mut car, &terrain, 8.0, dt);
            car
        });
        assert!(
            (a.pos() - b.pos()).length() < 0.1,
            "{:?} {:?}",
            a.pos(),
            b.pos()
        );
        assert!(a.velocity().length() < 0.05, "{:?}", a.velocity());
        assert!(b.velocity().length() < 0.05, "{:?}", b.velocity());
    }
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
