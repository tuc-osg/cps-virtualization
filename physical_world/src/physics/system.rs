use dimensioned::si;
use itertools::Itertools;
use log;
use vector3d::Vector3d;

use crate::physics::entity::Entity;
use crate::physics::interaction::interaction::Interaction;
use crate::utils::identity::Identity;

pub const PRECISION: f64 = 10e10;

#[derive(Clone)]
pub struct System {
    entities: Vec<Entity>,
    interactions: Vec<&'static dyn Interaction>,
    current_time: si::Second<f64>,
}

impl System {
    fn check_identities<I: Identity>(vec: &Vec<I>) -> bool {
        for idx0 in 0..vec.len() {
            for idx1 in idx0 + 1..vec.len() {
                if idx0 != idx1 && vec[idx0].get_identity() == vec[idx1].get_identity() {
                    return false;
                }
            }
        }
        true
    }

    pub fn new(
        entities: Vec<Entity>,
        interactions: Vec<&'static dyn Interaction>,
        current_time: si::Second<f64>,
    ) -> System {
        if !System::check_identities(&entities) {
            panic!("Identities for entities are not unique.")
        }
        if !System::check_identities(&interactions) {
            panic!("Identities for interactions are not unique.")
        }
        System {
            entities,
            interactions,
            current_time,
        }
    }

    pub fn get_current_time(&self) -> si::Second<f64> {
        self.current_time
    }

    pub fn get_entities(&self) -> &Vec<Entity> {
        &self.entities
    }

    pub fn get_momentum(&self) -> Vector3d<si::NewtonSecond<f64>> {
        self.entities.iter().fold(
            Vector3d::new(0.0 * si::NS, 0.0 * si::NS, 0.0 * si::NS),
            |momentum, e| momentum + e.get_momentum(),
        )
    }

    pub fn get_energy(&self) -> si::Joule<f64> {
        let kinetic_energy = self
            .entities
            .iter()
            .fold(0.0 * si::J, |energy, e| energy + e.get_kinetic_energy());
        let potential_energy = self
            .entities
            .iter()
            .permutations(2)
            .filter(|e| e[0] != e[1])
            .fold(si::J, |energy, e| energy + e[0].get_potential_energy(e[1]));
        potential_energy + kinetic_energy
    }

    pub fn next_state(&mut self, elapsed_time: si::Second<f64>) {
        let mut influences = Vec::new();
        // TODO: Look at Entities and possible neighbors, choose which interactions take place
        for interaction in &self.interactions {
            influences.extend(interaction.get_influences(&mut self.entities, elapsed_time));
        }
        // Graph = (V, E), V = Entities, E = Influences
        if !influences.is_empty() {
            println!(
                "TIME {:.03}, Step size={}\n",
                self.current_time, elapsed_time
            );
            for entity in &self.entities {
                println!("{}", entity);
            }
        }
        for entity in &mut self.entities {
            for influence in &influences {
                if entity.get_identity() == influence.get_receiver_id() {
                    println!("{}", influence);
                    entity.add_influence(&influence);
                }
            }
        }
        for entity in &mut self.entities {
            entity.evolve(elapsed_time);
        }
        for entity in &mut self.entities {
            entity.remove_influences();
        }
        self.current_time = self.current_time + elapsed_time;
        log::trace!("SYSTEM STATE at {}:\n{}", self.current_time, self);
    }
}

impl std::fmt::Display for System {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{}\n", "=".repeat(80))?;
        write!(f, "SYSTEM MOMENTUM: {}\n", self.get_momentum())?;
        write!(f, "SYSTEM ENERGY: {}\n", self.get_energy())?;
        write!(f, "{}\n", "=".repeat(80))?;
        for entity in &self.entities {
            match write!(f, "{}", entity) {
                Err(e) => return Err(e),
                Ok(_) => (),
            }
        }
        Ok(())
    }
}
