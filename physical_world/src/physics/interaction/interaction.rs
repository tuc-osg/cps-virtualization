use dimensioned::si;
use crate::physics::entity::Entity;
use crate::physics::state::state_influence::StateInfluence;
use crate::utils::identity::Identity;

pub trait Interaction {
    fn get_identifier(&self) -> &'static str;
    fn is_neighbor(&self, transmitter: &Entity, receiver: &Entity) -> bool;
    fn init(&self, source: &Entity, neighbors: Vec<&Entity>, step_size: si::Second<f64>) -> Vec<StateInfluence>;
    fn react(
        &self,
        reactor: &Entity,
        neighbors: Vec<&Entity>,
        influence: StateInfluence,
        step_size: si::Second<f64>
    ) -> Vec<StateInfluence>;

    fn print(&self, transmitter: &Entity, receiver: &Entity, influence: &StateInfluence) {
        println!(
            "\n[{}]({} -> {}) -> {{\n{}}}\n",
            self.get_identifier(),
            transmitter.get_identity(),
            receiver.get_identity(),
            influence
        );
    }

    fn log(&self, transmitter: &Entity, receiver: &Entity, influence: &StateInfluence) {
        log::trace!(
            "\n[{}]({} -> {}) -> {{\n{}}}\n",
            self.get_identifier(),
            transmitter.get_identity(),
            receiver.get_identity(),
            influence
        );
    }

    fn dfs(
        &self,
        world: &Vec<Entity>,
        influence_stack: &mut Vec<StateInfluence>,
        step_size: si::Second<f64>
    ) -> Vec<StateInfluence> {
        let mut final_influences = Vec::new();
        while let Some(influence) = influence_stack.pop() {
            if influence.get_transmitter_id() == influence.get_receiver_id() {
                final_influences.push(influence);
            } else {
                let receiver_idx_opt = world
                    .iter()
                    .position(|e| e.get_identity() == influence.get_receiver_id());
                let receiver = match receiver_idx_opt {
                    Some(idx) => &world[idx],
                    None => panic!("Receiver of influence not found."),
                };
                let neighbors = world
                    .iter()
                    .filter(|&e| e != receiver && self.is_neighbor(&receiver, e))
                    .collect::<Vec<&Entity>>();
                influence_stack.extend(self.react(receiver, neighbors, influence, step_size));
            }
        }
        final_influences
    }

    fn get_influences(&self, world: &Vec<Entity>, step_size: si::Second<f64>) -> Vec<StateInfluence> {
        let mut influences = Vec::new();
        for source in world.clone() {
            let neighbors = world
                .iter()
                .filter(|&e| *e != source && self.is_neighbor(&source, e))
                .collect::<Vec<&Entity>>();
            let mut influence_stack = self.init(&source, neighbors, step_size);
            influences.extend(self.dfs(world, &mut influence_stack, step_size));
        }
        influences
    }
}

impl Eq for &'static dyn Interaction {}

impl PartialEq for &'static dyn Interaction {
    fn eq(&self, other: &Self) -> bool {
        self.get_identifier() == other.get_identifier()
    }
}

impl Identity for &'static dyn Interaction {
    fn get_identity(&self) -> &'static str {
        self.get_identifier()
    }
}

#[cfg(test)]
mod force_transmission_tests {
    use super::*;
    use crate::physics::state::shape::{Shape, Sphere};
    use crate::physics::interaction::contact_forces::ContactForces;
    use crate::physics::state::state::State;
    use dimensioned::si;
    use vector3d::Vector3d;

    #[test]
    fn force_transmission_0() {
        let world = vec![
            Entity::new(
                "e0",
                State::new(
                    Vector3d::new(0.0, 0.0, 0.0) * si::M,
                    Vector3d::new(10.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(10.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
            Entity::new(
                "e1",
                State::new(
                    Vector3d::new(2.0, 0.0, 0.0) * si::M,
                    Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(0.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
            Entity::new(
                "e2",
                State::new(
                    Vector3d::new(4.0, 0.0, 0.0) * si::M,
                    Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(0.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
        ];
        let interaction = ContactForces;
        let mut expected_influences = vec![
            StateInfluence::force_influence(
                "e0",
                "e0",
                "e0",
                interaction.get_identifier(),
                Vector3d::new(-10.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e0",
                "e2",
                "e2",
                interaction.get_identifier(),
                Vector3d::new(10.0, 0.0, 0.0) * si::N,
            ),
        ];
        let res_influences = interaction.get_influences(&world, 1.0 * si::S);
        for res_influence in &res_influences {
            match expected_influences
                .iter()
                .position(|inf| res_influence == inf)
            {
                Some(index) => {
                    expected_influences.remove(index);
                }
                None => panic!("Not in expected influences:\n{}", res_influence),
            };
        }
        assert!(
            expected_influences.is_empty(),
            "Some expected influences were not in results."
        );
    }

    #[test]
    fn force_transmission_1() {
        let world = vec![
            Entity::new(
                "e0",
                State::new(
                    Vector3d::new(0.0, 0.0, 0.0) * si::M,
                    Vector3d::new(10.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(10.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
            Entity::new(
                "e1",
                State::new(
                    Vector3d::new(2.0, 0.0, 0.0) * si::M,
                    Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(0.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
            Entity::new(
                "e2",
                State::new(
                    Vector3d::new(4.0, 0.0, 0.0) * si::M,
                    Vector3d::new(-10.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(-10.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
        ];
        let interaction = ContactForces;
        let mut expected_influences = vec![
            StateInfluence::force_influence(
                "e0",
                "e0",
                "e0",
                interaction.get_identifier(),
                Vector3d::new(-10.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e0",
                "e2",
                "e2",
                interaction.get_identifier(),
                Vector3d::new(10.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e2",
                "e2",
                "e2",
                interaction.get_identifier(),
                Vector3d::new(10.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e2",
                "e0",
                "e0",
                interaction.get_identifier(),
                Vector3d::new(-10.0, 0.0, 0.0) * si::N,
            ),
        ];
        let res_influences = interaction.get_influences(&world, 1.0 * si::S);
        for res_influence in &res_influences {
            match expected_influences
                .iter()
                .position(|inf| res_influence == inf)
            {
                Some(index) => {
                    expected_influences.remove(index);
                }
                None => panic!("Not in expected influences:\n{}", res_influence),
            };
        }
        assert!(
            expected_influences.is_empty(),
            "Some expected influences were not in results."
        );
    }

    #[test]
    fn force_transmission_2() {
        let world = vec![
            Entity::new(
                "e0",
                State::new(
                    Vector3d::new(0.0, 0.0, 0.0) * si::M,
                    Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(10.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
            Entity::new(
                "e1",
                State::new(
                    Vector3d::new(2.0, 0.0, 0.0) * si::M,
                    Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(0.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
            Entity::new(
                "e2",
                State::new(
                    Vector3d::new(4.0, 0.0, 0.0) * si::M,
                    Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(-10.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
        ];
        let interaction = ContactForces;
        let mut expected_influences = vec![
            StateInfluence::force_influence(
                "e0",
                "e0",
                "e0",
                interaction.get_identifier(),
                Vector3d::new(-10.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e0",
                "e2",
                "e2",
                interaction.get_identifier(),
                Vector3d::new(10.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e2",
                "e2",
                "e2",
                interaction.get_identifier(),
                Vector3d::new(10.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e2",
                "e0",
                "e0",
                interaction.get_identifier(),
                Vector3d::new(-10.0, 0.0, 0.0) * si::N,
            ),
        ];
        let res_influences = interaction.get_influences(&world, 1.0 * si::S);
        for res_influence in &res_influences {
            match expected_influences
                .iter()
                .position(|inf| res_influence == inf)
            {
                Some(index) => {
                    expected_influences.remove(index);
                }
                None => panic!("Not in expected influences:\n{}", res_influence),
            };
        }
        assert!(
            expected_influences.is_empty(),
            "Some expected influences were not in results."
        );
    }
}

#[cfg(test)]
mod momentum_transition_tests {
    use super::*;
    use crate::physics::interaction::elastic_collision::ElasticCollision;
    use crate::physics::state::shape::{Shape, Sphere};
    use crate::physics::state::state::State;
    use dimensioned::si;
    use vector3d::Vector3d;

    #[test]
    fn momentum_transmission_0() {
        let world = vec![
            Entity::new(
                "e0",
                State::new(
                    Vector3d::new(0.0, 0.0, 0.0) * si::M,
                    Vector3d::new(10.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(10.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
            Entity::new(
                "e1",
                State::new(
                    Vector3d::new(2.0, 0.0, 0.0) * si::M,
                    Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(0.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
            Entity::new(
                "e2",
                State::new(
                    Vector3d::new(4.0, 0.0, 0.0) * si::M,
                    Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(0.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
        ];
        let interaction = ElasticCollision;
        let mut expected_influences = vec![
            StateInfluence::force_influence(
                "e0",
                "e0",
                "e0",
                interaction.get_identifier(),
                Vector3d::new(-100.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e0",
                "e2",
                "e2",
                interaction.get_identifier(),
                Vector3d::new(100.0, 0.0, 0.0) * si::N,
            ),
        ];
        let res_influences = interaction.get_influences(&world, 1.0 * si::S);
        for res_influence in &res_influences {
            match expected_influences
                .iter()
                .position(|inf| res_influence == inf)
            {
                Some(index) => {
                    expected_influences.remove(index);
                }
                None => panic!("Not in expected influences:\n{}", res_influence),
            };
        }
        assert!(
            expected_influences.is_empty(),
            "Some expected influences were not in results."
        );
    }

    #[test]
    fn momentum_transmission_1() {
        let world = vec![
            Entity::new(
                "e0",
                State::new(
                    Vector3d::new(0.0, 0.0, 0.0) * si::M,
                    Vector3d::new(10.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(10.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
            Entity::new(
                "e1",
                State::new(
                    Vector3d::new(2.0, 0.0, 0.0) * si::M,
                    Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(0.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
            Entity::new(
                "e2",
                State::new(
                    Vector3d::new(4.0, 0.0, 0.0) * si::M,
                    Vector3d::new(-10.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(-10.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
        ];
        let interaction = ElasticCollision;
        let mut expected_influences = vec![
            StateInfluence::force_influence(
                "e0",
                "e2",
                "e2",
                interaction.get_identifier(),
                Vector3d::new(100.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e0",
                "e0",
                "e0",
                interaction.get_identifier(),
                Vector3d::new(-100.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e2",
                "e0",
                "e0",
                interaction.get_identifier(),
                Vector3d::new(-100.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e2",
                "e2",
                "e2",
                interaction.get_identifier(),
                Vector3d::new(100.0, 0.0, 0.0) * si::N,
            ),
        ];
        let res_influences = interaction.get_influences(&world, 1.0 * si::S);
        for res_influence in &res_influences {
            match expected_influences
                .iter()
                .position(|inf| res_influence == inf)
            {
                Some(index) => {
                    expected_influences.remove(index);
                }
                None => panic!("Not in expected influences:\n{}", res_influence),
            };
        }
        assert!(
            expected_influences.is_empty(),
            "Some expected influences were not in results."
        );
    }

    #[test]
    fn momentum_transmission_2() {
        let world = vec![
            Entity::new(
                "e0",
                State::new(
                    Vector3d::new(0.0, 0.0, 0.0) * si::M,
                    Vector3d::new(10.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(10.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
            Entity::new(
                "e1",
                State::new(
                    Vector3d::new(2.0, 0.0, 0.0) * si::M,
                    Vector3d::new(-10.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(-10.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
        ];
        let interaction = ElasticCollision;
        let mut expected_influences = vec![
            StateInfluence::force_influence(
                "e0",
                "e1",
                "e1",
                interaction.get_identifier(),
                Vector3d::new(100.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e0",
                "e0",
                "e0",
                interaction.get_identifier(),
                Vector3d::new(-100.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e1",
                "e0",
                "e0",
                interaction.get_identifier(),
                Vector3d::new(-100.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e1",
                "e1",
                "e1",
                interaction.get_identifier(),
                Vector3d::new(100.0, 0.0, 0.0) * si::N,
            ),
        ];
        let res_influences = interaction.get_influences(&world, 1.0 * si::S);
        for res_influence in &res_influences {
            match expected_influences
                .iter()
                .position(|inf| res_influence == inf)
            {
                Some(index) => {
                    expected_influences.remove(index);
                }
                None => panic!("Not in expected influences:\n{}", res_influence),
            };
        }
        assert!(
            expected_influences.is_empty(),
            "Some expected influences were not in results."
        );
    }

    #[test]
    fn momentum_transmission_3() {
        let world = vec![
            Entity::new(
                "e0",
                State::new(
                    Vector3d::new(0.0, 0.0, 0.0) * si::M,
                    Vector3d::new(10.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(0.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
            Entity::new(
                "e1",
                State::new(
                    Vector3d::new(2.0, 0.0, 0.0) * si::M,
                    Vector3d::new(5.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(00.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
        ];
        let interaction = ElasticCollision;
        let mut expected_influences = vec![
            StateInfluence::force_influence(
                "e0",
                "e1",
                "e1",
                interaction.get_identifier(),
                Vector3d::new(50.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e0",
                "e0",
                "e0",
                interaction.get_identifier(),
                Vector3d::new(-50.0, 0.0, 0.0) * si::N,
            ),
        ];
        let res_influences = interaction.get_influences(&world, 1.0 * si::S);
        for res_influence in &res_influences {
            match expected_influences
                .iter()
                .position(|inf| res_influence == inf)
            {
                Some(index) => {
                    expected_influences.remove(index);
                }
                None => panic!("Not in expected influences:\n{}", res_influence),
            };
        }
        assert!(
            expected_influences.is_empty(),
            "Some expected influences were not in results."
        );
    }
}

#[cfg(test)]
mod force_momentum_transition_tests {
    use super::*;
    use crate::physics::interaction::elastic_collision::ElasticCollision;
    use crate::physics::state::shape::{Shape, Sphere};
    use crate::physics::interaction::contact_forces::ContactForces;
    use crate::physics::state::state::State;
    use dimensioned::si;
    use vector3d::Vector3d;

    #[test]
    fn force_momentum_transmission_0() {
        let world = vec![
            Entity::new(
                "e0",
                State::new(
                    Vector3d::new(0.0, 0.0, 0.0) * si::M,
                    Vector3d::new(10.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(10.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
            Entity::new(
                "e1",
                State::new(
                    Vector3d::new(2.0, 0.0, 0.0) * si::M,
                    Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(0.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
            Entity::new(
                "e2",
                State::new(
                    Vector3d::new(4.0, 0.0, 0.0) * si::M,
                    Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(0.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
        ];
        let ec = ElasticCollision;
        let cf = ContactForces;
        let mut expected_influences = vec![
            StateInfluence::force_influence(
                "e0",
                "e0",
                "e0",
                ec.get_identifier(),
                Vector3d::new(-100.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e0",
                "e0",
                "e0",
                cf.get_identifier(),
                Vector3d::new(-10.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e0",
                "e2",
                "e2",
                ec.get_identifier(),
                Vector3d::new(100.0, 0.0, 0.0) * si::N,
            ),

            StateInfluence::force_influence(
                "e0",
                "e2",
                "e2",
                cf.get_identifier(),
                Vector3d::new(10.0, 0.0, 0.0) * si::N,
            ),
        ];
        let mut res_influences = cf.get_influences(&world, 1.0 * si::S);
        res_influences.extend(ec.get_influences(&world, 1.0 * si::S));
        for res_influence in &res_influences {
            match expected_influences
                .iter()
                .position(|inf| res_influence == inf)
            {
                Some(index) => {
                    expected_influences.remove(index);
                }
                None => panic!("Not in expected influences:\n{}", res_influence),
            };
        }
        assert!(
            expected_influences.is_empty(),
            "Some expected influences were not in results."
        );
    }

    #[test]
    fn force_momentum_transmission_1() {
        let world = vec![
            Entity::new(
                "e0",
                State::new(
                    Vector3d::new(0.0, 0.0, 0.0) * si::M,
                    Vector3d::new(10.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(10.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
            Entity::new(
                "e1",
                State::new(
                    Vector3d::new(2.0, 0.0, 0.0) * si::M,
                    Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(0.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
        ];
        let ec = ElasticCollision;
        let cf = ContactForces;
        let mut expected_influences = vec![
            StateInfluence::force_influence(
                "e0",
                "e0",
                "e0",
                ec.get_identifier(),
                Vector3d::new(-100.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e0",
                "e1",
                "e1",
                ec.get_identifier(),
                Vector3d::new(100.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e0",
                "e0",
                "e0",
                cf.get_identifier(),
                Vector3d::new(-10.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e0",
                "e1",
                "e1",
                cf.get_identifier(),
                Vector3d::new(10.0, 0.0, 0.0) * si::N,
            ),
        ];
        let mut res_influences = cf.get_influences(&world, 1.0 * si::S);
        res_influences.extend(ec.get_influences(&world, 1.0 * si::S));
        for res_influence in &res_influences {
            match expected_influences
                .iter()
                .position(|inf| res_influence == inf)
            {
                Some(index) => {
                    expected_influences.remove(index);
                }
                None => panic!("Not in expected influences:\n{}", res_influence),
            };
        }
        assert!(
            expected_influences.is_empty(),
            "Some expected influences were not in results."
        );
    }

    #[test]
    fn force_momentum_transmission_2() {
        let world = vec![
            Entity::new(
                "e0",
                State::new(
                    Vector3d::new(0.0, 0.0, 0.0) * si::M,
                    Vector3d::new(10.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(10.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
            Entity::new(
                "e1",
                State::new(
                    Vector3d::new(2.0, 0.0, 0.0) * si::M,
                    Vector3d::new(-10.0, 0.0, 0.0) * si::MPS,
                    Vector3d::new(-10.0, 0.0, 0.0) * si::N,
                    10.0 * si::KG,
                    Shape::Sphere(Sphere {
                        radius: 1.0 * si::M,
                    }),
                ),
            ),
        ];
        let ec = ElasticCollision;
        let cf = ContactForces;
        let mut expected_influences = vec![
            StateInfluence::force_influence(
                "e0",
                "e0",
                "e0",
                ec.get_identifier(),
                Vector3d::new(-100.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e0",
                "e1",
                "e1",
                ec.get_identifier(),
                Vector3d::new(100.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e0",
                "e0",
                "e0",
                cf.get_identifier(),
                Vector3d::new(-10.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e0",
                "e1",
                "e1",
                cf.get_identifier(),
                Vector3d::new(10.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e1",
                "e1",
                "e1",
                ec.get_identifier(),
                Vector3d::new(100.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e1",
                "e0",
                "e0",
                ec.get_identifier(),
                Vector3d::new(-100.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e1",
                "e1",
                "e1",
                cf.get_identifier(),
                Vector3d::new(10.0, 0.0, 0.0) * si::N,
            ),
            StateInfluence::force_influence(
                "e1",
                "e0",
                "e0",
                cf.get_identifier(),
                Vector3d::new(-10.0, 0.0, 0.0) * si::N,
            ),
        ];
        let mut res_influences = cf.get_influences(&world, 1.0 * si::S);
        res_influences.extend(ec.get_influences(&world, 1.0 * si::S));
        for res_influence in &res_influences {
            match expected_influences
                .iter()
                .position(|inf| res_influence == inf)
            {
                Some(index) => {
                    expected_influences.remove(index);
                }
                None => panic!("Not in expected influences:\n{}", res_influence),
            };
        }
        assert!(
            expected_influences.is_empty(),
            "Some expected influences were not in results."
        );
    }
}