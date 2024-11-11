use dimensioned::si;
use vector3d::Vector3d;

use crate::physics::entity::Entity;
use crate::physics::interaction::interaction::Interaction;
use crate::physics::state::state_influence::StateInfluence;

use crate::physics::interaction::helpers::get_gravity_pull;

#[derive(Clone, Copy)]
pub struct Gravity;

impl Interaction for Gravity {
    // fn is_applicable(&self, _target: &Entity, _influencer: &Entity, _env: &Vec<&Entity>) -> bool {
    //     true
    // }

    // fn law(&self, target: &Entity, influencer: &Entity) -> StateInfluence {
    //     let state_influence = StateInfluence::new(
    //         Vector3d::new(0.0, 0.0, 0.0) * si::M,
    //         Vector3d::new(0.0, 0.0, 0.0) * si::M / si::S,
    //         get_gravity_pull(influencer, target),
    //         0.0 * si::KG,
    //         Shape::None,
    //         Vec::<EmitterPoint>::new(),
    //     );
    //     // self.print(target, influencer, state_influence);
    //     self.log(target, influencer, &state_influence);
    //     state_influence
    // }

    fn is_neighbor(&self, transmitter: &Entity, receiver: &Entity) -> bool {
        true
    }

    fn init(&self, source: &Entity, neighbors: Vec<&Entity>, step_size: si::Second<f64>) -> Vec<StateInfluence> {
        Vec::new()
    }

    fn react(
        &self,
        receiver: &Entity,
        neighbors: Vec<&Entity>,
        influence: StateInfluence,
        step_size: si::Second<f64>
    ) -> Vec<StateInfluence> {
        Vec::new()
    }

    fn get_identifier(&self) -> &'static str {
        "Gravity"
    }
}
