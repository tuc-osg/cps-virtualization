use dimensioned::{si, Sqrt};
use vector3d::Vector3d;

use crate::physics::entity::Entity;
use crate::physics::state::shape::Shape;

pub fn are_touching(entity0: &Entity, entity1: &Entity) -> bool {
    let s0 = entity0.get_state();
    let s1 = entity1.get_state();
    let loc0 = s0.get_location();
    let loc1 = s1.get_location();
    let dist = (loc0 - loc1).norm2().sqrt();
    let target_radius = match s0.get_shape() {
        Shape::Sphere(s) => s.radius,
        _ => panic!("Shape is not a Sphere."),
    };
    let influencer_radius = match s1.get_shape() {
        Shape::Sphere(s) => s.radius,
        _ => panic!("Shape is not a Sphere."),
    };
    dist <= target_radius + influencer_radius
}

fn get_normal_direction(from_entity: &Entity, to_entity: &Entity) -> Vector3d<si::Unitless<f64>> {
    let from = from_entity.get_state();
    let to = to_entity.get_state();
    let rel_location: Vector3d<si::Meter<f64>> = to.get_location() - from.get_location();
    let dist: si::Meter<f64> = rel_location.norm2().sqrt();
    rel_location / dist
}

fn get_force_val_in_direction(from_entity: &Entity, to_entity: &Entity) -> si::Newton<f64> {
    let from = from_entity.get_state();
    let force = from.get_net_force();
    let normal_direction = get_normal_direction(from_entity, to_entity);
    normal_direction.dot(force)
}

pub fn applies_force_in_direction(from_entity: &Entity, to_entity: &Entity) -> bool {
    let res = get_force_val_in_direction(from_entity, to_entity);
    res > (0.0 * si::N) && relative_velocity(from_entity, to_entity) <= (0.0 * si::M2PS)
    // res > 0.0 => forcing towards, < 0.0 => forcing away, == 0.0 => not forcing
    // Not forcing when moving away
}

pub fn get_force_in_direction(
    from_entity: &Entity,
    to_entity: &Entity,
) -> Vector3d<si::Newton<f64>> {
    if !applies_force_in_direction(from_entity, to_entity) {
        Vector3d::new(0.0, 0.0, 0.0) * si::N
    } else if !are_touching(from_entity, to_entity) {
        Vector3d::new(0.0, 0.0, 0.0) * si::N
    } else {
        let normal_direction = get_normal_direction(from_entity, to_entity);
        let force_in_direction = get_force_val_in_direction(from_entity, to_entity);
        normal_direction * force_in_direction
    }
}

fn relative_velocity(from_entity: &Entity, to_entity: &Entity) -> si::Meter2PerSecond<f64> {
    let from = from_entity.get_state();
    let to = to_entity.get_state();
    let rel_velocity = to.get_velocity() - from.get_velocity();
    let rel_location = to.get_location() - from.get_location();
    rel_velocity.dot(rel_location)
}

pub fn relatively_moves_towards(from_entity: &Entity, to_entity: &Entity) -> bool {
    relative_velocity(from_entity, to_entity) < (0.0 * si::M2PS)
    // < 0.0 => moving towards, > 0.0 => moving away, == 0.0 => not moving
}

fn get_velocity_val_in_direction(from_entity: &Entity, to_entity: &Entity) -> si::MeterPerSecond<f64> {
    let from = from_entity.get_state();
    let velocity = from.get_velocity();
    let normal_direction = get_normal_direction(from_entity, to_entity);
    normal_direction.dot(velocity)
}

// Moving towards other and distance gets smaller
pub fn moves_towards(from_entity: &Entity, to_entity: &Entity) -> bool {
    let res = get_velocity_val_in_direction(from_entity, to_entity);
    res > 0.0 * si::MPS && relatively_moves_towards(from_entity, to_entity)
}

pub fn get_velocity_diff_after_collision(
    from_entity: &Entity,
    to_entity: &Entity,
) -> Vector3d<si::MeterPerSecond<f64>> {
    let res = if !relatively_moves_towards(from_entity, to_entity) {
        Vector3d::new(0.0, 0.0, 0.0) * si::MPS
    } else if !are_touching(from_entity, to_entity) {
        Vector3d::new(0.0, 0.0, 0.0) * si::MPS
    } else {
        let from = from_entity.get_state();
        let to = to_entity.get_state();
        let rel_location = to.get_location() - from.get_location();
        let dist = rel_location.norm2().sqrt();
        let normal_direction = rel_location / dist;
        let reduced_system_mass = 1.0 / (1.0 / to.get_mass() + 1.0 / from.get_mass());
        let rel_velocity = to.get_velocity() - from.get_velocity();
        let impact_speed = normal_direction.dot(rel_velocity);
        let coefficient_of_restitution = 1.0;
        let impulse = (1.0 + coefficient_of_restitution) * reduced_system_mass * impact_speed;
        let velocity_diff: Vector3d<si::MeterPerSecond<f64>> =
            -normal_direction * (impulse / to.get_mass());
        velocity_diff
    };
    res
}

pub fn get_normal_force(from_entity: &Entity, to_entity: &Entity) -> Vector3d<si::Newton<f64>> {
    -get_force_in_direction(to_entity, from_entity)
}

pub fn get_gravity_pull(from_entity: &Entity, to_entity: &Entity) -> Vector3d<si::Newton<f64>> {
    let from = from_entity.get_state();
    let to = to_entity.get_state();
    let g = 6.67430e-11 * si::N * si::M2PKG / si::KG;
    let rel_location = from.get_location() - to.get_location();
    let force = rel_location * g * from.get_mass() * to.get_mass()
        / (rel_location.norm2() * rel_location.norm2().sqrt());
    force
}

#[cfg(test)]
mod test_helpers {
    use super::*;
    use crate::physics::entity::Entity;
    use crate::physics::state::shape::{Shape, Sphere};
    use crate::physics::state::state::State;
    use dimensioned::si;
    use vector3d::Vector3d;

    const TEST_PRECISION: u128 = 10e10 as u128; // Precision in digits

    #[test]
    fn test_move_towards0() {
        let e0 = Entity::new(
            "e0",
            State::new(
                Vector3d::new(0.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere( Sphere { radius: 1.0 * si::M } ),
            ),
        );
        let e1 = Entity::new(
            "e0",
            State::new(
                Vector3d::new(1.0, 0.0, 0.0) * si::M,
                Vector3d::new(10.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere( Sphere { radius: 1.0 * si::M } ),
            ),
        );
        assert!(moves_towards(&e0, &e1) == false, "Expected {}, got {}.", false, true);
    }

    #[test]
    fn test_move_towards1() {
        let e0 = Entity::new(
            "e0",
            State::new(
                Vector3d::new(0.0, 0.0, 0.0) * si::M,
                Vector3d::new(10.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere( Sphere { radius: 1.0 * si::M } ),
            ),
        );
        let e1 = Entity::new(
            "e1",
            State::new(
                Vector3d::new(1.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere( Sphere { radius: 1.0 * si::M } ),
            ),
        );
        assert!(moves_towards(&e0, &e1) == true, "Expected {}, got {}.", true, false);
    }

    #[test]
    fn test_move_towards2() {
        let e0 = Entity::new(
            "e0",
            State::new(
                Vector3d::new(0.0, 0.0, 0.0) * si::M,
                Vector3d::new(10.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere( Sphere { radius: 1.0 * si::M } ),
            ),
        );
        let e1 = Entity::new(
            "e1",
            State::new(
                Vector3d::new(1.0, 0.0, 0.0) * si::M,
                Vector3d::new(10.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere( Sphere { radius: 1.0 * si::M } ),
            ),
        );
        assert!(moves_towards(&e0, &e1) == false, "Expected {}, got {}.", false, true);
    }

    #[test]
    fn test_move_towards3() {
        let e0 = Entity::new(
            "e0",
            State::new(
                Vector3d::new(0.0, 0.0, 0.0) * si::M,
                Vector3d::new(5.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere( Sphere { radius: 1.0 * si::M } ),
            ),
        );
        let e1 = Entity::new(
            "e1",
            State::new(
                Vector3d::new(1.0, 0.0, 0.0) * si::M,
                Vector3d::new(10.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere( Sphere { radius: 1.0 * si::M } ),
            ),
        );
        assert!(moves_towards(&e0, &e1) == false, "Expected {}, got {}.", false, true);
    }

    #[test]
    fn test_move_towards4() {
        let e0 = Entity::new(
            "e0",
            State::new(
                Vector3d::new(0.0, 0.0, 0.0) * si::M,
                Vector3d::new(10.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere( Sphere { radius: 1.0 * si::M } ),
            ),
        );
        let e1 = Entity::new(
            "e1",
            State::new(
                Vector3d::new(1.0, 0.0, 0.0) * si::M,
                Vector3d::new(5.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere( Sphere { radius: 1.0 * si::M } ),
            ),
        );
        assert!(moves_towards(&e0, &e1) == true, "Expected {}, got {}.", true, false);
    }

    #[test]
    fn test_are_touching() {
        fn test_touch_entities(
            loc0: Vector3d<si::Meter<f64>>,
            radius0: si::Meter<f64>,
            loc1: Vector3d<si::Meter<f64>>,
            radius1: si::Meter<f64>,
            expected_result: bool,
        ) {
            let e0 = Entity::new(
                "test entity 0",
                State::new(
                    loc0,
                    Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(0.0, 0.0, 0.0) * si::N,
                    0.0 * si::KG,
                    Shape::Sphere(Sphere { radius: radius0 }),
                ),
            );
            let e1 = Entity::new(
                "test entity 1",
                State::new(
                    loc1,
                    Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(0.0, 0.0, 0.0) * si::N,
                    0.0 * si::KG,
                    Shape::Sphere(Sphere { radius: radius1 }),
                ),
            );
            let res0 = are_touching(&e0, &e1);
            let res1 = are_touching(&e1, &e0);
            assert!(
                res0 == expected_result && res1 == expected_result,
                "TEST FAILED (WAS {} (for e0->e1) and {} (for e1->e0), EXPECTED {}) FOR:\n{}\n{}",
                res0,
                res1,
                expected_result,
                e0,
                e1
            );
        }

        test_touch_entities(
            Vector3d::new(1.0, 0.0, 0.0) * si::M,
            1.0 * si::M,
            Vector3d::new(-1.0, 0.0, 0.0) * si::M,
            1.0 * si::M,
            true,
        );
        test_touch_entities(
            Vector3d::new(0.9, 0.0, 0.0) * si::M,
            1.0 * si::M,
            Vector3d::new(-0.9, 0.0, 0.0) * si::M,
            1.0 * si::M,
            true,
        );
        test_touch_entities(
            Vector3d::new(-1.1, 0.0, 0.0) * si::M,
            1.0 * si::M,
            Vector3d::new(1.1, 0.0, 0.0) * si::M,
            1.0 * si::M,
            false,
        );
        test_touch_entities(
            Vector3d::new(1.0, 1.0, 1.0) * si::M,
            1.0 * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            1.0 * si::M,
            true,
        );
        test_touch_entities(
            Vector3d::new(-1.0, -1.0, -1.0) * si::M,
            1.0 * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            1.0 * si::M,
            true,
        );
        test_touch_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            1.0 * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            1.0 * si::M,
            true,
        );
        test_touch_entities(
            Vector3d::new(1.0, 1.0, 1.0) * si::M,
            1.0 * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::M,
            1.0 * si::M,
            true,
        );
        test_touch_entities(
            Vector3d::new(-1.0, -1.0, -1.0) * si::M,
            1.0 * si::M,
            Vector3d::new(-1.0, -1.0, -1.0) * si::M,
            1.0 * si::M,
            true,
        );
        test_touch_entities(
            Vector3d::new(0.5, 0.5, 0.5) * si::M,
            1.0 * si::M,
            Vector3d::new(-0.5, -0.5, -0.5) * si::M,
            1.0 * si::M,
            true,
        );
        test_touch_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            1.0 * si::M,
            Vector3d::new(-0.5, -0.5, -0.5) * si::M,
            1.0 * si::M,
            true,
        );
        test_touch_entities(
            Vector3d::new(1.0, 1.0, 1.0) * si::M,
            1.0 * si::M,
            Vector3d::new(-2.0, -2.5, -2.5) * si::M,
            1.0 * si::M,
            false,
        );
        test_touch_entities(
            Vector3d::new(-110.0, 70.0, -80.0) * si::M,
            1.0 * si::M,
            Vector3d::new(-2.0, -2.5, -2.5) * si::M,
            1.0 * si::M,
            false,
        );
    }

    #[test]
    fn test_relatively_moves_towards() {
        fn test_move_entities(
            loc0: Vector3d<si::Meter<f64>>,
            vel0: Vector3d<si::MeterPerSecond<f64>>,
            loc1: Vector3d<si::Meter<f64>>,
            vel1: Vector3d<si::MeterPerSecond<f64>>,
            expected_result: bool,
        ) {
            let e0 = Entity::new(
                "test entity 0",
                State::new(
                    loc0,
                    vel0,
                    Vector3d::new(0.0, 0.0, 0.0) * si::N,
                    0.0 * si::KG,
                    Shape::None,
                ),
            );
            let e1 = Entity::new(
                "test entity 1",
                State::new(
                    loc1,
                    vel1,
                    Vector3d::new(0.0, 0.0, 0.0) * si::N,
                    0.0 * si::KG,
                    Shape::None,
                ),
            );
            let res0 = relatively_moves_towards(&e0, &e1);
            let res1 = relatively_moves_towards(&e1, &e0);
            assert!(
                res0 == expected_result && res1 == expected_result,
                "TEST FAILED (WAS {} (for e0->e1) and {} (for e1->e0), EXPECTED {}) FOR:\n{}\n{}",
                res0,
                res1,
                expected_result,
                e0,
                e1
            );
        }
        test_move_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::MPS,
            Vector3d::new(1.0, 1.0, 1.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
            true,
        );
        test_move_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::MPS,
            Vector3d::new(1.0, 1.0, 1.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::MPS,
            false,
        );
        test_move_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.1, 1.1, 1.1) * si::MPS,
            Vector3d::new(1.0, 1.0, 1.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::MPS,
            true,
        );
        test_move_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::MPS,
            Vector3d::new(1.0, 1.0, 1.0) * si::M,
            Vector3d::new(1.1, 1.1, 1.1) * si::MPS,
            false,
        );
        test_move_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            -Vector3d::new(1.0, 1.0, 1.0) * si::MPS,
            -Vector3d::new(1.0, 1.0, 1.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
            true,
        );
        test_move_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            -Vector3d::new(1.0, 1.0, 1.0) * si::MPS,
            -Vector3d::new(1.0, 1.0, 1.0) * si::M,
            -Vector3d::new(1.0, 1.0, 1.0) * si::MPS,
            false,
        );
        test_move_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            -Vector3d::new(1.1, 1.1, 1.1) * si::MPS,
            -Vector3d::new(1.0, 1.0, 1.0) * si::M,
            -Vector3d::new(1.0, 1.0, 1.0) * si::MPS,
            true,
        );
        test_move_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            -Vector3d::new(1.0, 1.0, 1.0) * si::MPS,
            -Vector3d::new(1.0, 1.0, 1.0) * si::M,
            -Vector3d::new(1.1, 1.1, 1.1) * si::MPS,
            false,
        );
        test_move_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            -Vector3d::new(1.0, 1.0, 1.0) * si::MPS,
            Vector3d::new(1.0, 1.0, 1.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
            false,
        );
        test_move_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            -Vector3d::new(1.0, 1.0, 1.0) * si::MPS,
            -Vector3d::new(1.0, 1.0, 1.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::MPS,
            true,
        );
        test_move_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            -Vector3d::new(1.1, 1.1, 1.1) * si::MPS,
            Vector3d::new(1.0, 1.0, 1.0) * si::M,
            -Vector3d::new(1.0, 1.0, 1.0) * si::MPS,
            false,
        );
        test_move_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            -Vector3d::new(1.0, 1.0, 1.0) * si::MPS,
            -Vector3d::new(1.0, 1.0, 1.0) * si::M,
            Vector3d::new(1.1, 1.1, 1.1) * si::MPS,
            true,
        );
        test_move_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::MPS,
            Vector3d::new(1.0, 0.0, 0.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
            true,
        );
        test_move_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            -Vector3d::new(1.0, 1.0, 1.0) * si::MPS,
            Vector3d::new(1.0, 0.0, 0.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
            false,
        );
        test_move_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::MPS,
            -Vector3d::new(1.0, 0.0, 0.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
            false,
        );
        test_move_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            -Vector3d::new(1.0, 1.0, 1.0) * si::MPS,
            -Vector3d::new(1.0, 0.0, 0.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
            true,
        );
        test_move_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
            Vector3d::new(1.0, 0.0, 0.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
            false,
        );
        test_move_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
            -Vector3d::new(1.0, 0.0, 0.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
            false,
        );
    }

    #[test]
    fn test_applies_force_in_direction() {
        fn test_force_entities(
            loc0: Vector3d<si::Meter<f64>>,
            force0: Vector3d<si::Newton<f64>>,
            loc1: Vector3d<si::Meter<f64>>,
            expected_result: bool,
        ) {
            let e0 = Entity::new(
                "test entity 0",
                State::new(
                    loc0,
                    Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                    force0,
                    0.0 * si::KG,
                    Shape::None,
                ),
            );
            let e1 = Entity::new(
                "test entity 1",
                State::new(
                    loc1,
                    Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(0.0, 0.0, 0.0) * si::N,
                    0.0 * si::KG,
                    Shape::None,
                ),
            );
            let res = applies_force_in_direction(&e0, &e1);
            assert!(
                res == expected_result,
                "TEST FAILED (WAS {}, EXPECTED {}) FOR:\n{}\n{}",
                res,
                expected_result,
                e0,
                e1
            );
        }
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::N,
            Vector3d::new(1.0, 1.0, 1.0) * si::M,
            true,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            -Vector3d::new(1.0, 1.0, 1.0) * si::N,
            Vector3d::new(1.0, 1.0, 1.0) * si::M,
            false,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::N,
            -Vector3d::new(1.0, 1.0, 1.0) * si::M,
            false,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::N,
            Vector3d::new(1.0, 0.0, 0.0) * si::M,
            true,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 0.0, 0.0) * si::N,
            Vector3d::new(1.0, 1.0, 1.0) * si::M,
            true,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            -Vector3d::new(1.0, 1.0, 1.0) * si::N,
            -Vector3d::new(1.0, 0.0, 0.0) * si::M,
            true,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            -Vector3d::new(1.0, 0.0, 0.0) * si::N,
            -Vector3d::new(1.0, 1.0, 1.0) * si::M,
            true,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            -Vector3d::new(1.0, 0.0, 0.0) * si::N,
            Vector3d::new(1.0, 1.0, 1.0) * si::M,
            false,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 0.0, 0.0) * si::N,
            -Vector3d::new(1.0, 1.0, 1.0) * si::M,
            false,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::N,
            -Vector3d::new(1.0, 1.0, 1.0) * si::M,
            false,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::N,
            Vector3d::new(1.0, 1.0, 1.0) * si::M,
            false,
        );
    }

    #[test]
    fn test_get_force_in_direction() {
        fn test_force_entities(
            loc0: Vector3d<si::Meter<f64>>,
            force0: Vector3d<si::Newton<f64>>,
            loc1: Vector3d<si::Meter<f64>>,
            expected_result: Vector3d<si::Newton<f64>>,
        ) {
            let e0 = Entity::new(
                "test entity 0",
                State::new(
                    loc0,
                    Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                    force0,
                    0.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 2.0 * si::M,
                    }),
                ),
            );
            let e1 = Entity::new(
                "test entity 1",
                State::new(
                    loc1,
                    Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(0.0, 0.0, 0.0) * si::N,
                    0.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 2.0 * si::M,
                    }),
                ),
            );
            let res = get_force_in_direction(&e0, &e1);
            let diff = (res - expected_result).norm2().sqrt();
            let rounded_diff = (diff.value_unsafe * TEST_PRECISION as f64) as u128;
            assert!(
                rounded_diff == 0,
                "TEST FAILED (WAS {}, EXPECTED {}) FOR:\n{}\n{}",
                res,
                expected_result,
                e0,
                e1
            );
        }
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::N,
            Vector3d::new(1.0, 1.0, 1.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::N,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::N,
            Vector3d::new(1.0, 1.0, 1.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::N,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::N,
            -Vector3d::new(1.0, 1.0, 1.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::N,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            -Vector3d::new(1.0, 1.0, 1.0) * si::N,
            -Vector3d::new(1.0, 1.0, 1.0) * si::M,
            -Vector3d::new(1.0, 1.0, 1.0) * si::N,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::N,
            Vector3d::new(1.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 0.0, 0.0) * si::N,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::N,
            Vector3d::new(0.0, 1.0, 0.0) * si::M,
            Vector3d::new(0.0, 1.0, 0.0) * si::N,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::N,
            Vector3d::new(0.0, 0.0, 1.0) * si::M,
            Vector3d::new(0.0, 0.0, 1.0) * si::N,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::N,
            Vector3d::new(0.0, 1.0, 1.0) * si::M,
            Vector3d::new(0.0, 1.0, 1.0) * si::N,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::N,
            Vector3d::new(2.5, 2.5, 2.5) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::N,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::N,
            Vector3d::new(1.5, 1.5, 1.5) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::N,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::N,
            Vector3d::new(0.0, 2.0, 2.0) * si::M,
            Vector3d::new(0.0, 1.0, 1.0) * si::N,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::N,
            Vector3d::new(10.0, 2.0, 2.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::N,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            -Vector3d::new(1.0, 1.0, 1.0) * si::N,
            -Vector3d::new(0.0, 2.0, 2.0) * si::M,
            -Vector3d::new(0.0, 1.0, 1.0) * si::N,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            -Vector3d::new(1.0, 1.0, 1.0) * si::N,
            Vector3d::new(0.0, 2.0, 2.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::N,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            -Vector3d::new(1.0, 1.0, 1.0) * si::N,
            Vector3d::new(0.0, -2.0, 2.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::N,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(-1.0, 1.0, 1.0) * si::N,
            Vector3d::new(0.0, 1.5, 1.5) * si::M,
            Vector3d::new(0.0, 1.0, 1.0) * si::N,
        );
        test_force_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::N,
            -Vector3d::new(1.0, 0.0, 0.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::N,
        );
        let loc0 = Vector3d::new(0.0, 0.0, 0.0) * si::M;
        let force0 = Vector3d::new(1.0, 1.0, 1.0) * si::N;
        let loc1 = Vector3d::new(0.0, 1.0, 2.0) * si::M;
        let dir = (loc1 - loc0) / (loc1 - loc0).norm2().sqrt();
        let expected_force = dir * force0.dot(dir);
        test_force_entities(loc0, force0, loc1, expected_force);
        let loc0 = Vector3d::new(0.0, 0.0, 0.0) * si::M;
        let force0 = -Vector3d::new(1.0, 1.0, 1.0) * si::N;
        let loc1 = -Vector3d::new(0.0, 1.0, 2.0) * si::M;
        let dir = (loc1 - loc0) / (loc1 - loc0).norm2().sqrt();
        let expected_force = dir * force0.dot(dir);
        test_force_entities(loc0, force0, loc1, expected_force);
        let loc0 = Vector3d::new(0.0, 0.0, 0.0) * si::M;
        let force0 = -Vector3d::new(-1.0, 1.0, -2.0) * si::N;
        let loc1 = -Vector3d::new(-1.0, 2.3, -1.0) * si::M;
        let dir = (loc1 - loc0) / (loc1 - loc0).norm2().sqrt();
        let expected_force = dir * force0.dot(dir);
        test_force_entities(loc0, force0, loc1, expected_force);
    }

    #[test]
    fn test_get_velocity_diff_after_collision() {
        fn test_collision_entities(
            loc0: Vector3d<si::Meter<f64>>,
            vel0: Vector3d<si::MeterPerSecond<f64>>,
            mass0: si::Kilogram<f64>,
            loc1: Vector3d<si::Meter<f64>>,
            vel1: Vector3d<si::MeterPerSecond<f64>>,
            mass1: si::Kilogram<f64>,
            expected_result: Vector3d<si::MeterPerSecond<f64>>,
        ) {
            let e0 = Entity::new(
                "test entity 0",
                State::new(
                    loc0,
                    vel0,
                    Vector3d::new(0.0, 0.0, 0.0) * si::N,
                    mass0,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            );
            let e1 = Entity::new(
                "test entity 1",
                State::new(
                    loc1,
                    vel1,
                    Vector3d::new(0.0, 0.0, 0.0) * si::N,
                    mass1,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            );
            let res = get_velocity_diff_after_collision(&e0, &e1);
            let diff = (res - expected_result).norm2().sqrt();
            let rounded_diff = (diff.value_unsafe * TEST_PRECISION as f64) as u128;
            assert!(
                rounded_diff == 0,
                "TEST FAILED (WAS {}, EXPECTED {}) FOR:\n{}\n{}",
                res,
                expected_result,
                e0,
                e1
            );
        }
        test_collision_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
            10.0 * si::KG,
            Vector3d::new(1.0, 1.0, 1.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
            10.0 * si::KG,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
        );
        test_collision_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::MPS,
            10.0 * si::KG,
            Vector3d::new(10.0, 10.0, 10.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
            10.0 * si::KG,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
        );
        test_collision_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::MPS,
            10.0 * si::KG,
            Vector3d::new(1.0, 1.0, 1.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
            10.0 * si::KG,
            Vector3d::new(1.0, 1.0, 1.0) * si::MPS,
        );
        test_collision_entities(
            Vector3d::new(1.0, 1.0, 1.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
            10.0 * si::KG,
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::MPS,
            10.0 * si::KG,
            Vector3d::new(-1.0, -1.0, -1.0) * si::MPS,
        );
        test_collision_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::MPS,
            10.0 * si::KG,
            Vector3d::new(1.0, 1.0, 1.0) * si::M,
            Vector3d::new(-1.0, -1.0, -1.0) * si::MPS,
            10.0 * si::KG,
            Vector3d::new(2.0, 2.0, 2.0) * si::MPS,
        );
        test_collision_entities(
            Vector3d::new(1.0, 1.0, 1.0) * si::M,
            Vector3d::new(-1.0, -1.0, -1.0) * si::MPS,
            10.0 * si::KG,
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 1.0, 1.0) * si::MPS,
            10.0 * si::KG,
            Vector3d::new(-2.0, -2.0, -2.0) * si::MPS,
        );
        test_collision_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(2.0, 2.0, 2.0) * si::MPS,
            10.0 * si::KG,
            Vector3d::new(1.0, 1.0, 1.0) * si::M,
            Vector3d::new(-1.0, -1.0, -1.0) * si::MPS,
            10.0 * si::KG,
            Vector3d::new(3.0, 3.0, 3.0) * si::MPS,
        );
        test_collision_entities(
            Vector3d::new(1.0, 1.0, 1.0) * si::M,
            Vector3d::new(-1.0, -1.0, -1.0) * si::MPS,
            10.0 * si::KG,
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(2.0, 2.0, 2.0) * si::MPS,
            10.0 * si::KG,
            Vector3d::new(-3.0, -3.0, -3.0) * si::MPS,
        );
        test_collision_entities(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 0.0, 0.0) * si::MPS,
            30.0 * si::KG,
            Vector3d::new(1.0, 0.0, 0.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
            10.0 * si::KG,
            Vector3d::new(1.5, 0.0, 0.0) * si::MPS,
        );
        test_collision_entities(
            Vector3d::new(1.0, 0.0, 0.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
            10.0 * si::KG,
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(1.0, 0.0, 0.0) * si::MPS,
            30.0 * si::KG,
            Vector3d::new(-0.5, 0.0, 0.0) * si::MPS,
        );
    }

    #[test]
    fn test_get_normal_force() {}

    #[test]
    fn test_get_gravity_pull() {}
}
