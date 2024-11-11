use dimensioned::{si, Sqrt};
use vector3d::Vector3d;

use crate::physics::entity::Entity;
use crate::physics::interaction::helpers::{
    are_touching, get_velocity_diff_after_collision, get_force_in_direction, moves_towards
};
use crate::physics::interaction::interaction::Interaction;
use crate::physics::state::state::State;
use crate::physics::state::state_influence::StateInfluence;
use crate::physics::system::PRECISION;
use crate::utils::identity::Identity;

use super::helpers::applies_force_in_direction;

#[derive(Clone, Copy)]
pub struct ElasticCollision;

impl Interaction for ElasticCollision {
    fn is_neighbor(&self, transmitter: &Entity, receiver: &Entity) -> bool {
        are_touching(transmitter, receiver)
    }

    fn init(&self, source: &Entity, neighbors: Vec<&Entity>, step_size: si::Second<f64>) -> Vec<StateInfluence> {
        let mut influences = Vec::new();
        let mut forces = Vector3d::new(0.0, 0.0, 0.0) * si::N;
        for neigh in neighbors
            .iter()
            .filter(|n| moves_towards(source, n))
        {
            let collision_vel = if moves_towards(neigh, source) {
                let dummy = Entity::new(
                    neigh.get_identity(),
                    State::new(
                        neigh.get_state().get_location(),
                        Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                        neigh.get_state().get_net_force(),
                        neigh.get_state().get_mass(),
                        neigh.get_state().get_shape(),
                    ),
                );
                get_velocity_diff_after_collision(source, &dummy)
            } else {
                get_velocity_diff_after_collision(source, neigh)
            };
            if collision_vel.norm2().sqrt() * PRECISION > 1.0 * si::MPS {
                let force = collision_vel / step_size * neigh.get_state().get_mass();
                let influence = StateInfluence::force_influence(
                    source.get_identity(),
                    source.get_identity(),
                    neigh.get_identity(),
                    self.get_identifier(),
                    force,
                );
                forces = forces + force;
                influences.push(influence);
            }
        }
        if forces.norm2().sqrt() * PRECISION > 1.0 * si::N {
            let influence = StateInfluence::force_influence(
                source.get_identity(),
                source.get_identity(),
                source.get_identity(),
                self.get_identifier(),
                -forces,
            );
            influences.push(influence);
        }
        influences
    }

    fn react(
        &self,
        reactor: &Entity,
        neighbors: Vec<&Entity>,
        influence: StateInfluence,
        _step_size: si::Second<f64>
    ) -> Vec<StateInfluence> {
        if reactor.get_identity() != influence.get_receiver_id() {
            panic!(
                "Reactor id {} does not match influence receiver id {}.",
                reactor.get_identity(),
                influence.get_receiver_id()
            );
        }
        let inf_entity = Entity::new(
            "-",
            State::new(
                reactor.get_state().get_location(),
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                influence.get_state_change().get_net_force(),
                reactor.get_state().get_mass(),
                reactor.get_state().get_shape(),
            ),
        );
        let mut influences = Vec::new();
        let mut forces = Vector3d::new(0.0, 0.0, 0.0) * si::N;
        for neigh in neighbors
            .iter()
            .filter(|n| applies_force_in_direction(&inf_entity, n))
        {
            let force = get_force_in_direction(&inf_entity, neigh);
            if force.norm2().sqrt() * PRECISION > 1.0 * si::N {
                let new_influence = StateInfluence::force_influence(
                    influence.get_source_id(),
                    reactor.get_identity(),
                    neigh.get_identity(),
                    self.get_identifier(),
                    force,
                );
                forces = forces + force;
                influences.push(new_influence);
            }
        }
        let reactor_force = influence.get_state_change().get_net_force() - forces;
        if reactor_force.norm2().sqrt() * PRECISION > 1.0 * si::N {
            let self_influence = StateInfluence::force_influence(
                influence.get_source_id(),
                reactor.get_identity(),
                reactor.get_identity(),
                self.get_identifier(),
                reactor_force,
            );
            influences.push(self_influence);
        }
        influences
    }

    fn get_identifier(&self) -> &'static str {
        "elastic collision"
    }
}

#[cfg(test)]
mod elastic_collision {
    use super::*;
    use crate::physics::entity::Entity;
    use crate::physics::state::shape::{Shape, Sphere};
    use crate::physics::state::state::State;
    use dimensioned::si;

    #[test]
    fn test_is_neighbor_0() {
        let e0 = Entity::new(
            "test entity 0",
            State::new(
                Vector3d::new(0.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let e1 = Entity::new(
            "test entity 1",
            State::new(
                Vector3d::new(5.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let expected_res = false;
        let elastic_collision = ElasticCollision;
        let res = elastic_collision.is_neighbor(&e0, &e1);
        assert!(
            res == expected_res,
            "TEST FAILED (WAS {} (for e0->e1), EXPECTED {}) FOR:\n{}\n{}",
            res,
            expected_res,
            e0,
            e1
        );
    }

    #[test]
    fn test_is_neighbor_1() {
        let e0 = Entity::new(
            "test entity 0",
            State::new(
                Vector3d::new(0.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let e1 = Entity::new(
            "test entity 1",
            State::new(
                Vector3d::new(-5.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let expected_res = false;
        let elastic_collision = ElasticCollision;
        let res = elastic_collision.is_neighbor(&e0, &e1);
        assert!(
            res == expected_res,
            "TEST FAILED (WAS {} (for e0->e1), EXPECTED {}) FOR:\n{}\n{}",
            res,
            expected_res,
            e0,
            e1
        );
    }

    #[test]
    fn test_is_neighbor_2() {
        let e0 = Entity::new(
            "test entity 0",
            State::new(
                Vector3d::new(20.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let e1 = Entity::new(
            "test entity 1",
            State::new(
                Vector3d::new(22.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let expected_res = true;
        let elastic_collision = ElasticCollision;
        let res = elastic_collision.is_neighbor(&e0, &e1);
        assert!(
            res == expected_res,
            "TEST FAILED (WAS {} (for e0->e1), EXPECTED {}) FOR:\n{}\n{}",
            res,
            expected_res,
            e0,
            e1
        );
    }

    #[test]
    fn test_is_neighbor_3() {
        let e0 = Entity::new(
            "test entity 0",
            State::new(
                Vector3d::new(20.0, 0.0, 0.0) * si::M,
                Vector3d::new(10.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let e1 = Entity::new(
            "test entity 1",
            State::new(
                Vector3d::new(22.0, 0.0, 0.0) * si::M,
                Vector3d::new(10.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let expected_res = true;
        let elastic_collision = ElasticCollision;
        let res = elastic_collision.is_neighbor(&e0, &e1);
        assert!(
            res == expected_res,
            "TEST FAILED (WAS {} (for e0->e1), EXPECTED {}) FOR:\n{}\n{}",
            res,
            expected_res,
            e0,
            e1
        );
    }

    #[test]
    fn test_is_neighbor_4() {
        let e0 = Entity::new(
            "test entity 0",
            State::new(
                Vector3d::new(20.0, 0.0, 0.0) * si::M,
                Vector3d::new(10.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let e1 = Entity::new(
            "test entity 1",
            State::new(
                Vector3d::new(22.0, 0.0, 0.0) * si::M,
                Vector3d::new(-5.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let expected_res = true;
        let elastic_collision = ElasticCollision;
        let res = elastic_collision.is_neighbor(&e0, &e1);
        assert!(
            res == expected_res,
            "TEST FAILED (WAS {} (for e0->e1), EXPECTED {}) FOR:\n{}\n{}",
            res,
            expected_res,
            e0,
            e1
        );
    }

    #[test]
    fn test_is_neighbor_5() {
        let e0 = Entity::new(
            "test entity 0",
            State::new(
                Vector3d::new(20.0, 0.0, 0.0) * si::M,
                Vector3d::new(10.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let e1 = Entity::new(
            "test entity 1",
            State::new(
                Vector3d::new(22.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let expected_res = true;
        let elastic_collision = ElasticCollision;
        let res = elastic_collision.is_neighbor(&e0, &e1);
        assert!(
            res == expected_res,
            "TEST FAILED (WAS {} (for e0->e1), EXPECTED {}) FOR:\n{}\n{}",
            res,
            expected_res,
            e0,
            e1
        );
    }

    #[test]
    fn test_is_neighbor_6() {
        let e0 = Entity::new(
            "test entity 0",
            State::new(
                Vector3d::new(20.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let e1 = Entity::new(
            "test entity 1",
            State::new(
                Vector3d::new(22.0, 0.0, 0.0) * si::M,
                Vector3d::new(10.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let expected_res = true;
        let elastic_collision = ElasticCollision;
        let res = elastic_collision.is_neighbor(&e0, &e1);
        assert!(
            res == expected_res,
            "TEST FAILED (WAS {} (for e0->e1), EXPECTED {}) FOR:\n{}\n{}",
            res,
            expected_res,
            e0,
            e1
        );
    }

    #[test]
    fn test_is_neighbor_7() {
        let e0 = Entity::new(
            "test entity 0",
            State::new(
                Vector3d::new(20.0, 0.0, 0.0) * si::M,
                Vector3d::new(11.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let e1 = Entity::new(
            "test entity 1",
            State::new(
                Vector3d::new(22.0, 0.0, 0.0) * si::M,
                Vector3d::new(10.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let expected_res = true;
        let elastic_collision = ElasticCollision;
        let res = elastic_collision.is_neighbor(&e0, &e1);
        assert!(
            res == expected_res,
            "TEST FAILED (WAS {} (for e0->e1), EXPECTED {}) FOR:\n{}\n{}",
            res,
            expected_res,
            e0,
            e1
        );
    }

    #[test]
    fn test_is_neighbor_8() {
        let e0 = Entity::new(
            "test entity 0",
            State::new(
                Vector3d::new(20.0, 0.0, 0.0) * si::M,
                Vector3d::new(-10.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let e1 = Entity::new(
            "test entity 1",
            State::new(
                Vector3d::new(22.0, 0.0, 0.0) * si::M,
                Vector3d::new(-11.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let expected_res = true;
        let elastic_collision = ElasticCollision;
        let res = elastic_collision.is_neighbor(&e0, &e1);
        assert!(
            res == expected_res,
            "TEST FAILED (WAS {} (for e0->e1), EXPECTED {}) FOR:\n{}\n{}",
            res,
            expected_res,
            e0,
            e1
        );
    }

    #[test]
    fn test_is_neighbor_9() {
        let e0 = Entity::new(
            "test entity 0",
            State::new(
                Vector3d::new(-1.0, 0.0, 0.0) * si::M,
                Vector3d::new(-10.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let e1 = Entity::new(
            "test entity 1",
            State::new(
                Vector3d::new(1.0, 0.0, 0.0) * si::M,
                Vector3d::new(-11.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                0.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let expected_res = true;
        let elastic_collision = ElasticCollision;
        let res = elastic_collision.is_neighbor(&e0, &e1);
        assert!(
            res == expected_res,
            "TEST FAILED (WAS {} (for e0->e1), EXPECTED {}) FOR:\n{}\n{}",
            res,
            expected_res,
            e0,
            e1
        );
    }

    #[test]
    fn test_empty_init_0() {
        let source = Entity::new(
            "source",
            State::new(
                Vector3d::new(0.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let neighbors = vec![];
        let elastic_collision = ElasticCollision;
        let res = elastic_collision.init(&source, neighbors.clone(), 1.0 * si::S);
        assert!(
            res.is_empty(),
            "For no neighbors, resulting influences should have been empty for source:\n{}",
            source
        );
    }

    #[test]
    fn test_empty_init_1() {
        let source = Entity::new(
            "source",
            State::new(
                Vector3d::new(0.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let n0 = Entity::new(
            "neighbor 0",
            State::new(
                Vector3d::new(10.0, 0.0, 0.0) * si::M,
                Vector3d::new(-10.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let neighbors = vec![&n0];
        let elastic_collision = ElasticCollision;
        let res = elastic_collision.init(&source, neighbors.clone(), 1.0 * si::S);
        assert!(
            res.is_empty(),
            "For no neighbors, resulting influences should have been empty for source:\n{}",
            source
        );
    }

    #[test]
    fn test_empty_init_2() {
        let source = Entity::new(
            "source",
            State::new(
                Vector3d::new(0.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let n0 = Entity::new(
            "neighbor 0",
            State::new(
                Vector3d::new(2.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let n1 = Entity::new(
            "neighbor 1",
            State::new(
                Vector3d::new(-2.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let n2 = Entity::new(
            "neighbor 2",
            State::new(
                Vector3d::new(2.0, 0.0, 0.0) * si::M,
                Vector3d::new(10.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(10.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let n3 = Entity::new(
            "neighbor 3",
            State::new(
                Vector3d::new(-2.0, 0.0, 0.0) * si::M,
                Vector3d::new(-10.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(-10.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let neighbors = vec![&n0, &n1, &n2, &n3];
        let elastic_collision = ElasticCollision;
        let res = elastic_collision.init(&source, neighbors.clone(), 1.0 * si::S);
        assert!(
            res.is_empty(),
            "For no neighbors, resulting influences should have been empty for source:\n{}",
            source
        );
    }

    #[test]
    fn test_init_source() {
        let source = Entity::new(
            "source",
            State::new(
                Vector3d::new(0.0, 0.0, 0.0) * si::M,
                Vector3d::new(10.0, -10.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let n0 = Entity::new(
            "neighbor 0",
            State::new(
                Vector3d::new(2.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let n1 = Entity::new(
            "neighbor 1",
            State::new(
                Vector3d::new(0.0, -2.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let elastic_collision = ElasticCollision;
        let neighbors = vec![&n0, &n1];
        let mut res = elastic_collision.init(&source, neighbors.clone(), 1.0 * si::S);
        let res_self = res.pop().unwrap();
        let res_other1 = res.pop().unwrap();
        let res_other0 = res.pop().unwrap();
        let expected_res_self = StateInfluence::force_influence(
            source.get_identity(),
            source.get_identity(),
            source.get_identity(),
            elastic_collision.get_identifier(),
            Vector3d::new(-100.0, 100.0, 0.0) * si::N,
        );
        let expected_res_other1 = StateInfluence::force_influence(
            source.get_identity(),
            source.get_identity(),
            n1.get_identity(),
            elastic_collision.get_identifier(),
            Vector3d::new(0.0, -100.0, 0.0) * si::N,
        );
        let expected_res_other0 = StateInfluence::force_influence(
            source.get_identity(),
            source.get_identity(),
            n0.get_identity(),
            elastic_collision.get_identifier(),
            Vector3d::new(100.0, 0.0, 0.0) * si::N,
        );
        assert!(
            res_self == expected_res_self,
            "Expected\n{}\ngot\n{}",
            expected_res_self,
            res_self
        );
        assert!(
            res_other0 == expected_res_other0,
            "Expected\n{}\ngot\n{}",
            expected_res_other0,
            res_other0
        );
        assert!(
            res_other1 == expected_res_other1,
            "Expected\n{}\ngot\n{}",
            expected_res_other1,
            res_other1
        );
        assert!(
            res.is_empty(),
            "Resulting influences should have been empty."
        );
    }

    #[test]
    fn test_init_source_different_weight() {
        let source = Entity::new(
            "source",
            State::new(
                Vector3d::new(0.0, 0.0, 0.0) * si::M,
                Vector3d::new(10.0, 10.0, 0.0) * si::MPS,
                Vector3d::new(10.0, 10.0, 0.0) * si::N,
                30.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let n0 = Entity::new(
            "neighbor 0",
            State::new(
                Vector3d::new(2.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let n1 = Entity::new(
            "neighbor 1",
            State::new(
                Vector3d::new(0.0, 2.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let elastic_collision = ElasticCollision;
        let neighbors = vec![&n0, &n1];
        let mut res = elastic_collision.init(&source, neighbors.clone(), 1.0 * si::S);
        let res_self = res.pop().unwrap();
        let res_other1 = res.pop().unwrap();
        let res_other0 = res.pop().unwrap();
        let expected_res_self = StateInfluence::force_influence(
            source.get_identity(),
            source.get_identity(),
            source.get_identity(),
            elastic_collision.get_identifier(),
            Vector3d::new(-150.0, -150.0, 0.0) * si::N,
        );
        let expected_res_other1 = StateInfluence::force_influence(
            source.get_identity(),
            source.get_identity(),
            n1.get_identity(),
            elastic_collision.get_identifier(),
            Vector3d::new(0.0, 150.0, 0.0) * si::N,
        );
        let expected_res_other0 = StateInfluence::force_influence(
            source.get_identity(),
            source.get_identity(),
            n0.get_identity(),
            elastic_collision.get_identifier(),
            Vector3d::new(150.0, 0.0, 0.0) * si::N,
        );
        assert!(
            res_self == expected_res_self,
            "Expected\n{}\ngot\n{}",
            expected_res_self,
            res_self
        );
        assert!(
            res_other0 == expected_res_other0,
            "Expected\n{}\ngot\n{}",
            expected_res_other0,
            res_other0
        );
        assert!(
            res_other1 == expected_res_other1,
            "Expected\n{}\ngot\n{}",
            expected_res_other1,
            res_other1
        );
        assert!(
            res.is_empty(),
            "Resulting influences should have been empty."
        );
    }

    #[test]
    fn test_init_others() {
        let source = Entity::new(
            "source",
            State::new(
                Vector3d::new(0.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let n0 = Entity::new(
            "neighbor 0",
            State::new(
                Vector3d::new(2.0, 0.0, 0.0) * si::M,
                Vector3d::new(-10.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let n1 = Entity::new(
            "neighbor 1",
            State::new(
                Vector3d::new(0.0, -2.0, 0.0) * si::M,
                Vector3d::new(0.0, 10.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let elastic_collision = ElasticCollision;
        let neighbors = vec![&n0, &n1];
        let res = elastic_collision.init(&source, neighbors.clone(), 1.0 * si::S);
        // let res_self = res.pop().unwrap();
        // let res_other1 = res.pop().unwrap();
        // let res_other0 = res.pop().unwrap();
        // let expected_res_self = StateInfluence::force_influence(
        //     source.get_identity(),
        //     source.get_identity(),
        //     source.get_identity(),
        //     elastic_collision.get_identifier(),
        //     Vector3d::new(-100.0, 100.0, 0.0) * si::N,
        // );
        // let expected_res_other1 = StateInfluence::force_influence(
        //     source.get_identity(),
        //     source.get_identity(),
        //     n1.get_identity(),
        //     elastic_collision.get_identifier(),
        //     Vector3d::new(0.0, -100.0, 0.0) * si::N,
        // );
        // let expected_res_other0 = StateInfluence::force_influence(
        //     source.get_identity(),
        //     source.get_identity(),
        //     n0.get_identity(),
        //     elastic_collision.get_identifier(),
        //     Vector3d::new(100.0, 0.0, 0.0) * si::N,
        // );
        // assert!(
        //     res_self == expected_res_self,
        //     "Expected\n{}\ngot\n{}",
        //     expected_res_self,
        //     res_self
        // );
        // assert!(
        //     res_other0 == expected_res_other0,
        //     "Expected\n{}\ngot\n{}",
        //     expected_res_other0,
        //     res_other0
        // );
        // assert!(
        //     res_other1 == expected_res_other1,
        //     "Expected\n{}\ngot\n{}",
        //     expected_res_other1,
        //     res_other1
        // );
        assert!(
            res.is_empty(),
            "Resulting influences should have been empty."
        );
    }

    #[test]
    fn test_init_others_different_weight() {
        let source = Entity::new(
            "source",
            State::new(
                Vector3d::new(0.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let n0 = Entity::new(
            "neighbor 0",
            State::new(
                Vector3d::new(2.0, 0.0, 0.0) * si::M,
                Vector3d::new(-10.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(-10.0, 0.0, 0.0) * si::N,
                30.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let elastic_collision = ElasticCollision;
        // let expected_res_self = StateInfluence::force_influence(
        //     source.get_identity(),
        //     source.get_identity(),
        //     source.get_identity(),
        //     elastic_collision.get_identifier(),
        //     Vector3d::new(-150.0, 0.0, 0.0) * si::N,
        // );
        // let expected_res_other0 = StateInfluence::force_influence(
        //     source.get_identity(),
        //     source.get_identity(),
        //     n0.get_identity(),
        //     elastic_collision.get_identifier(),
        //     Vector3d::new(150.0, 0.0, 0.0) * si::N,
        // );
        let neighbors = vec![&n0];
        let res = elastic_collision.init(&source, neighbors.clone(), 1.0 * si::S);
        // let res_self = res.pop().unwrap();
        // let res_other0 = res.pop().unwrap();
        // assert!(
        //     res_self == expected_res_self,
        //     "Expected\n{}\ngot\n{}",
        //     expected_res_self,
        //     res_self
        // );
        // assert!(
        //     res_other0 == expected_res_other0,
        //     "Expected\n{}\ngot\n{}",
        //     expected_res_other0,
        //     res_other0
        // );
        assert!(
            res.is_empty(),
            "Resulting influences should have been empty."
        );
    }

    #[test]
    fn test_empty_react_0() {
        let reactor = Entity::new(
            "reactor",
            State::new(
                Vector3d::new(4.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let elastic_collision = ElasticCollision;
        let influence = StateInfluence::force_influence(
            "source",
            "transmitter",
            "reactor",
            elastic_collision.get_identifier(),
            Vector3d::new(-100.0, 0.0, 0.0) * si::N,
        );
        let neighbors = vec![];
        let mut res = elastic_collision.react(&reactor, neighbors.clone(), influence.clone(), 1.0 * si::S);
        let expected_res_self = StateInfluence::force_influence(
            influence.get_source_id(),
            reactor.get_identity(),
            reactor.get_identity(),
            elastic_collision.get_identifier(),
            Vector3d::new(-100.0, 0.0, 0.0) * si::N,
        );
        let res_self = res.pop().unwrap();
        assert!(
            res_self == expected_res_self,
            "For no neighbors, resulting influence should be added to state of reactor."
        );
        assert!(
            res.is_empty(),
            "Resulting influences should have been empty."
        );
    }

    #[test]
    fn test_empty_react_1() {
        let reactor = Entity::new(
            "reactor",
            State::new(
                Vector3d::new(4.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let elastic_collision = ElasticCollision;
        let influence = StateInfluence::force_influence(
            "source",
            "transmitter",
            "reactor",
            elastic_collision.get_identifier(),
            Vector3d::new(-100.0, 0.0, 0.0) * si::N,
        );
        let n0 = Entity::new(
            "neighbor 0",
            State::new(
                Vector3d::new(10.0, 0.0, 0.0) * si::M,
                Vector3d::new(-10.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let n1 = Entity::new(
            "neighbor 1",
            State::new(
                Vector3d::new(-10.0, 0.0, 0.0) * si::M,
                Vector3d::new(10.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let n2 = Entity::new(
            "neighbor 2",
            State::new(
                Vector3d::new(1.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let neighbors = vec![&n0, &n1, &n2];
        let mut res = elastic_collision.react(&reactor, neighbors.clone(), influence.clone(), 1.0 * si::S);
        let expected_res_self = StateInfluence::force_influence(
            influence.get_source_id(),
            reactor.get_identity(),
            reactor.get_identity(),
            elastic_collision.get_identifier(),
            Vector3d::new(-100.0, 0.0, 0.0) * si::N,
        );
        let res_self = res.pop().unwrap();
        assert!(
            res_self == expected_res_self,
            "For no neighbors, resulting influence should be added to state of reactor."
        );
        assert!(
            res.is_empty(),
            "Resulting influences should have been empty."
        );
    }

    #[test]
    fn test_react_single_neigh_0() {
        let reactor = Entity::new(
            "reactor",
            State::new(
                Vector3d::new(4.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let elastic_collision = ElasticCollision;
        let influence = StateInfluence::force_influence(
            "source",
            "transmitter",
            "reactor",
            elastic_collision.get_identifier(),
            Vector3d::new(-100.0, 0.0, 0.0) * si::N,
        );
        let n0 = Entity::new(
            "neighbor 0",
            State::new(
                Vector3d::new(2.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let neighbors = vec![&n0];
        let mut res = elastic_collision.react(&reactor, neighbors.clone(), influence.clone(), 1.0 * si::S);
        let expected_res_other = StateInfluence::force_influence(
            influence.get_source_id(),
            reactor.get_identity(),
            n0.get_identity(),
            elastic_collision.get_identifier(),
            Vector3d::new(-100.0, 0.0, 0.0) * si::N,
        );
        let res_other = res.pop().unwrap();
        assert!(
            res_other == expected_res_other,
            "Expected\n{}\ngot\n{}",
            expected_res_other,
            res_other
        );
        assert!(
            res.is_empty(),
            "Resulting influences should have been empty."
        );
    }

    #[test]
    fn test_react_single_neigh_1() {
        let reactor = Entity::new(
            "reactor",
            State::new(
                Vector3d::new(4.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let elastic_collision = ElasticCollision;
        let influence = StateInfluence::force_influence(
            "source",
            "transmitter",
            "reactor",
            elastic_collision.get_identifier(),
            Vector3d::new(-100.0, 0.0, 0.0) * si::N,
        );
        let n0 = Entity::new(
            "neighbor 0",
            State::new(
                Vector3d::new(3.0, 1.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let neighbors = vec![&n0];
        let mut res = elastic_collision.react(&reactor, neighbors.clone(), influence.clone(), 1.0 * si::S);
        let expected_res_self = StateInfluence::force_influence(
            influence.get_source_id(),
            reactor.get_identity(),
            reactor.get_identity(),
            elastic_collision.get_identifier(),
            Vector3d::new(-50.0, -50.0, 0.0) * si::N,
        );
        let expected_res_other = StateInfluence::force_influence(
            influence.get_source_id(),
            reactor.get_identity(),
            n0.get_identity(),
            elastic_collision.get_identifier(),
            Vector3d::new(-50.0, 50.0, 0.0) * si::N,
        );
        let res_self = res.pop().unwrap();
        let res_other = res.pop().unwrap();
        assert!(
            res_self == expected_res_self,
            " Expected\n{}\ngot\n{}",
            expected_res_self,
            res_self
        );
        assert!(
            res_other == expected_res_other,
            " Expected\n{}\ngot\n{}",
            expected_res_other,
            res_other
        );
        assert!(
            res.is_empty(),
            "Resulting influences should have been empty."
        );
    }

    #[test]
    fn test_react_single_neigh_2() {
        let reactor = Entity::new(
            "reactor",
            State::new(
                Vector3d::new(3.0, 1.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let elastic_collision = ElasticCollision;
        let influence = StateInfluence::force_influence(
            "source",
            "transmitter",
            "reactor",
            elastic_collision.get_identifier(),
            Vector3d::new(100.0, 0.0, 0.0) * si::N,
        );
        let n0 = Entity::new(
            "neighbor 0",
            State::new(
                Vector3d::new(4.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let neighbors = vec![&n0];
        let mut res = elastic_collision.react(&reactor, neighbors.clone(), influence.clone(), 1.0 * si::S);
        let expected_res_self = StateInfluence::force_influence(
            influence.get_source_id(),
            reactor.get_identity(),
            reactor.get_identity(),
            elastic_collision.get_identifier(),
            Vector3d::new(50.0, 50.0, 0.0) * si::N,
        );
        let expected_res_other = StateInfluence::force_influence(
            influence.get_source_id(),
            reactor.get_identity(),
            n0.get_identity(),
            elastic_collision.get_identifier(),
            Vector3d::new(50.0, -50.0, 0.0) * si::N,
        );
        let res_self = res.pop().unwrap();
        let res_other = res.pop().unwrap();
        assert!(
            res_self == expected_res_self,
            "Expected\n{}\ngot\n{}",
            expected_res_self,
            res_self
        );
        assert!(
            res_other == expected_res_other,
            "Expected\n{}\ngot\n{}",
            expected_res_other,
            res_other
        );
        assert!(
            res.is_empty(),
            "Resulting influences should have been empty."
        );
    }

    #[test]
    fn test_react_single_neigh_3() {
        let reactor = Entity::new(
            "reactor",
            State::new(
                Vector3d::new(3.0, 1.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let elastic_collision = ElasticCollision;
        let influence = StateInfluence::force_influence(
            "source",
            "transmitter",
            "reactor",
            elastic_collision.get_identifier(),
            Vector3d::new(100.0, -100.0, 0.0) * si::N,
        );
        let n0 = Entity::new(
            "neighbor 0",
            State::new(
                Vector3d::new(4.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let neighbors = vec![&n0];
        let mut res = elastic_collision.react(&reactor, neighbors.clone(), influence.clone(), 1.0 * si::S);
        let expected_res_other = StateInfluence::force_influence(
            influence.get_source_id(),
            reactor.get_identity(),
            n0.get_identity(),
            elastic_collision.get_identifier(),
            Vector3d::new(100.0, -100.0, 0.0) * si::N,
        );
        let res_other = res.pop().unwrap();
        assert!(
            res_other == expected_res_other,
            "Expected\n{}\ngot\n{}",
            expected_res_other,
            res_other
        );
        assert!(
            res.is_empty(),
            "Resulting influences should have been empty."
        );
    }

    #[test]
    fn test_react_single_neigh_4() {
        let reactor = Entity::new(
            "reactor",
            State::new(
                Vector3d::new(-3.0, -1.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let elastic_collision = ElasticCollision;
        let influence = StateInfluence::force_influence(
            "source",
            "transmitter",
            "reactor",
            elastic_collision.get_identifier(),
            Vector3d::new(-100.0, 100.0, 0.0) * si::N,
        );
        let n0 = Entity::new(
            "neighbor 0",
            State::new(
                Vector3d::new(-4.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let neighbors = vec![&n0];
        let mut res = elastic_collision.react(&reactor, neighbors.clone(), influence.clone(), 1.0 * si::S);
        let expected_res_other = StateInfluence::force_influence(
            influence.get_source_id(),
            reactor.get_identity(),
            n0.get_identity(),
            elastic_collision.get_identifier(),
            Vector3d::new(-100.0, 100.0, 0.0) * si::N,
        );
        let res_other = res.pop().unwrap();
        assert!(
            res_other == expected_res_other,
            "Expected\n{}\ngot\n{}",
            expected_res_other,
            res_other
        );
        assert!(
            res.is_empty(),
            "Resulting influences should have been empty."
        );
    }

    #[test]
    fn test_multi_neigh_0() {
        let reactor = Entity::new(
            "reactor",
            State::new(
                Vector3d::new(2.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let elastic_collision = ElasticCollision;
        let influence = StateInfluence::force_influence(
            "source",
            "transmitter",
            "reactor",
            elastic_collision.get_identifier(),
            Vector3d::new(100.0, 0.0, 0.0) * si::N,
        );
        let n0 = Entity::new(
            "neighbor 0",
            State::new(
                Vector3d::new(3.0, 1.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let n1 = Entity::new(
            "neighbor 1",
            State::new(
                Vector3d::new(3.0, -1.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let neighbors = vec![&n0, &n1];
        let mut res = elastic_collision.react(&reactor, neighbors.clone(), influence.clone(), 1.0 * si::S);
        let expected_res_other0 = StateInfluence::force_influence(
            influence.get_source_id(),
            reactor.get_identity(),
            n0.get_identity(),
            elastic_collision.get_identifier(),
            Vector3d::new(50.0, 50.0, 0.0) * si::N,
        );
        let expected_res_other1 = StateInfluence::force_influence(
            influence.get_source_id(),
            reactor.get_identity(),
            n1.get_identity(),
            elastic_collision.get_identifier(),
            Vector3d::new(50.0, -50.0, 0.0) * si::N,
        );
        let res_other1 = res.pop().unwrap();
        let res_other0 = res.pop().unwrap();
        assert!(
            res_other0 == expected_res_other0,
            "Expected\n{}\ngot\n{}",
            expected_res_other0,
            res_other0
        );
        assert!(
            res_other1 == expected_res_other1,
            "Expected\n{}\ngot\n{}",
            expected_res_other1,
            res_other1
        );
        assert!(
            res.is_empty(),
            "Resulting influences should have been empty."
        );
    }

    #[test]
    fn test_multi_neigh_1() {
        let reactor = Entity::new(
            "reactor",
            State::new(
                Vector3d::new(0.0, -3.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let elastic_collision = ElasticCollision;
        let influence = StateInfluence::force_influence(
            "source",
            "transmitter",
            "reactor",
            elastic_collision.get_identifier(),
            Vector3d::new(0.0, -100.0, 0.0) * si::N,
        );
        let n0 = Entity::new(
            "neighbor 0",
            State::new(
                Vector3d::new(1.0, -4.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let n1 = Entity::new(
            "neighbor 1",
            State::new(
                Vector3d::new(-1.0, -4.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                Vector3d::new(0.0, 0.0, 0.0) * si::N,
                10.0 * si::KG,
                Shape::Sphere(Sphere {
                    radius: 1.0 * si::M,
                }),
            ),
        );
        let neighbors = vec![&n0, &n1];
        let mut res = elastic_collision.react(&reactor, neighbors.clone(), influence.clone(), 1.0 * si::S);
        let expected_res_other_0 = StateInfluence::force_influence(
            influence.get_source_id(),
            reactor.get_identity(),
            n0.get_identity(),
            elastic_collision.get_identifier(),
            Vector3d::new(50.0, -50.0, 0.0) * si::N,
        );
        let expected_res_other_1 = StateInfluence::force_influence(
            influence.get_source_id(),
            reactor.get_identity(),
            n1.get_identity(),
            elastic_collision.get_identifier(),
            Vector3d::new(-50.0, -50.0, 0.0) * si::N,
        );
        let res_other_1 = res.pop().unwrap();
        let res_other_0 = res.pop().unwrap();
        assert!(
            res_other_0 == expected_res_other_0,
            "Expected\n{}\ngot\n{}",
            expected_res_other_0,
            res_other_0
        );
        assert!(
            res_other_1 == expected_res_other_1,
            "Expected\n{}\ngot\n{}",
            expected_res_other_1,
            res_other_1
        );
        assert!(
            res.is_empty(),
            "Resulting influences should have been empty."
        );
    }
}
