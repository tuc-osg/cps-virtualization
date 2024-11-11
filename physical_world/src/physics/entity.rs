use dimensioned::{si, Sqrt};
use vector3d::Vector3d;

use crate::physics::state::state::State;
use crate::physics::state::state_influence::StateInfluence;
use crate::utils::identity::Identity;

#[derive(Clone, Debug)]
pub struct Entity {
    identifier: &'static str,
    state: State,
    influences: Vec<StateInfluence>,
}

impl Entity {
    pub fn new(identifier: &'static str, state: State) -> Entity {
        Entity {
            identifier,
            state,
            influences: Vec::new(),
        }
    }

    pub fn get_state(&self) -> &State {
        &self.state
    }

    pub fn get_momentum(&self) -> Vector3d<si::NewtonSecond<f64>> {
        self.state.get_velocity() * self.state.get_mass()
    }

    pub fn get_kinetic_energy(&self) -> si::Joule<f64> {
        (self.state.get_mass() * self.state.get_velocity().dot(self.state.get_velocity())) / 2.0
    }

    pub fn get_potential_energy(&self, other: &Entity) -> si::Joule<f64> {
        use crate::physics::interaction::helpers::get_gravity_pull;
        let rel_location = self.state.get_location() - other.state.get_location();
        let distance = rel_location.norm2().sqrt();
        let force = get_gravity_pull(other, self).norm2().sqrt();
        force * distance
    }

    pub fn evolve(&mut self, elapsed_time: si::Second<f64>) {
        for influence in &self.influences {
            self.state = &self.state + &influence.get_state_change();
        }
        self.state.evolve(elapsed_time);
    }

    pub fn add_influence(&mut self, influence: &StateInfluence) {
        self.influences.push(influence.clone());
    }

    pub fn remove_influences(&mut self) {
        for influence in &self.influences {
            self.state = &self.state - &influence.get_state_change();
        }
        self.influences = Vec::new();
    }
}

impl Identity for Entity {
    fn get_identity(&self) -> &'static str {
        self.identifier
    }
}

impl PartialEq for Entity {
    fn eq(&self, other: &Self) -> bool {
        self.identifier == other.identifier
    }
}

impl std::fmt::Display for Entity {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "\t{}\n{}", self.identifier, self.state)
    }
}
