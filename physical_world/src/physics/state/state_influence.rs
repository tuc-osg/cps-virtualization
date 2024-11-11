use dimensioned::si;
use vector3d::Vector3d;

use crate::physics::state::shape::Shape;
use crate::physics::state::state::State;

#[derive(Clone, PartialEq, Debug)]
pub struct StateInfluence {
    source_id: &'static str,
    transmitter_id: &'static str,
    receiver_id: &'static str,
    interaction_id: &'static str,
    state_change: State,
}

impl StateInfluence {
    pub fn new(
        source_id: &'static str,
        transmitter_id: &'static str,
        receiver_id: &'static str,
        interaction_id: &'static str,
        state_change: State,
    ) -> StateInfluence {
        StateInfluence {
            source_id,
            transmitter_id,
            receiver_id,
            interaction_id,
            state_change,
        }
    }

    pub fn force_influence(
        source_id: &'static str,
        transmitter_id: &'static str,
        receiver_id: &'static str,
        interaction_id: &'static str,
        force: Vector3d<si::Newton<f64>>,
    ) -> StateInfluence {
        StateInfluence {
            source_id,
            transmitter_id,
            receiver_id,
            interaction_id,
            state_change: State::new(
                Vector3d::new(0.0, 0.0, 0.0) * si::M,
                Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
                force,
                0.0 * si::KG,
                Shape::None,
            ),
        }
    }

    pub fn get_state_change(&self) -> State {
        self.state_change.clone()
    }

    pub fn get_source_id(&self) -> &'static str {
        self.source_id
    }

    pub fn get_transmitter_id(&self) -> &'static str {
        self.transmitter_id
    }

    pub fn get_receiver_id(&self) -> &'static str {
        self.receiver_id
    }

    pub fn get_interaction_id(&self) -> &'static str {
        self.interaction_id
    }
}

impl std::fmt::Display for StateInfluence {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "{} --{}--> {}\n{}",
            self.transmitter_id, self.source_id, self.receiver_id, self.state_change
        )
    }
}
