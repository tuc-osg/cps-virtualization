use dimensioned::si;
use vector3d::Vector3d;

use physical_machine::physics::entity::Entity;
use physical_machine::physics::state::state::State;
use physical_machine::physics::state::shape::{Shape, Sphere};
use physical_machine::physics::system::System;

mod common;

use common::simulation::{plot_results, write_csv};
use common::simulation::Simulation;
use common::interactions::INTERACTIONS;

#[test]
pub fn test_wall_collisions() {
    let name = "wall_collisions";
    let mut log_filename = "log/".to_owned();
    log_filename.push_str(&name);
    log_filename.push_str(".log");
    common::setup();
    common::logging::init_log(&log_filename);
    let e1 = Entity::new(
        "A",
        State::new(
            Vector3d::new(1000.0, 0.0, 0.0) * si::M,
            Vector3d::new(-100.0, 0.0, 0.0) * si::MPS,
            Vector3d::new(0.0, 0.0, 0.0) * si::N,
            10.0 * si::KG,
            Shape::Sphere(Sphere {
                radius: 1.0 * si::M,
            }),
        ),
    );

    let ground = Entity::new(
        "Wall",
        State::new(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
            Vector3d::new(0.0, 0.0, 0.0) * si::N,
            10_000.0 * si::KG,
            Shape::Sphere(Sphere {
                radius: 500.0 * si::M,
            }),
        ),
    );
    let sim = Simulation {
        simulation_time: 30.0 * si::S,
        time_step: 0.1 * si::S,
        system: System::new(
            vec![e1, ground],
            INTERACTIONS.to_vec(),
            0.0 * si::S,
        ),
    };
    let history = sim.run(false);
    let mut img_filename = "img/".to_owned();
    img_filename.push_str(&name);
    img_filename.push_str(".png");
    plot_results(&history, &img_filename, &name);
    let mut csv_filename = "csv/".to_owned();
    csv_filename.push_str(&name);
    csv_filename.push_str(".csv");
    write_csv(&history, &csv_filename);
}
