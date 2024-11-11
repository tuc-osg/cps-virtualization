use dimensioned::si;
use vector3d::Vector3d;

use physical_machine::physics::entity::Entity;
use physical_machine::physics::state::state::State;
use physical_machine::physics::system::System;
use physical_machine::physics::state::shape::{Shape, Sphere};

mod common;

use common::simulation::{plot_results, write_csv};
use common::simulation::Simulation;
use common::interactions::INTERACTIONS;

#[test]
pub fn test_multi_gravity() {
    let name = "multi_gravity";
    let mut log_filename = "log/".to_owned();
    log_filename.push_str(&name);
    log_filename.push_str(".log");
    common::setup();
    common::logging::init_log(&log_filename);
    let e1 = Entity::new(
        "A",
        State::new(
            Vector3d::new(5.0, 0.0, 0.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
            Vector3d::new(0.0, 0.0, 0.0) * si::N,
            100.0 * si::KG,
            Shape::Sphere(Sphere {
                radius: 1.0 * si::M,
            }),
        ),
    );

    let ground1 = Entity::new(
        "Ground1",
        State::new(
            Vector3d::new(10.0, 0.0, 0.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
            Vector3d::new(0.0, 0.0, 0.0) * si::N,
            4.0e5 * si::KG,
            Shape::Sphere(Sphere {
                radius: 1.0 * si::M,
            }),
        ),
    );

    let ground2 = Entity::new(
        "Ground2",
        State::new(
            Vector3d::new(0.0, 0.0, 0.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
            Vector3d::new(0.0, 0.0, 0.0) * si::N,
            4.0e5 * si::KG,
            Shape::Sphere(Sphere {
                radius: 1.0 * si::M,
            }),
        ),
    );

    let sim = Simulation {
        simulation_time: 70_000.0 * si::S,
        time_step: 0.3 * si::S,
        system: System::new(
            vec![ground2, ground1, e1],
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
