use physical_machine::physics::interaction::interaction::Interaction;
use physical_machine::physics::interaction::elastic_collision::ElasticCollision;
use physical_machine::physics::interaction::contact_forces::ContactForces;
// use physical_machine::physics::interaction::gravity::Gravity;

pub const INTERACTIONS: [&'static dyn Interaction; 2] = [
    &ContactForces,
    &ElasticCollision,
    //&Gravity,
];