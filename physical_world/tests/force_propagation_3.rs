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
fn test_force_propagation_3() {
    let name = "force_propagation_3";
    let mut log_filename = "log/".to_owned();
    log_filename.push_str(&name);
    log_filename.push_str(".log");
    common::setup();
    common::logging::init_log(&log_filename);
    let radius = 2.0 * si::M;
    let e1 = Entity::new(
        "A",
        State::new(
            Vector3d::new(30.0, 0.0, 0.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
            Vector3d::new(00.0, 0.0, 0.0) * si::N,
            100.0 * si::KG,
            Shape::Sphere(Sphere { radius }),
        ),
    );

    let e2 = Entity::new(
        "B",
        State::new(
            Vector3d::new(40.0, 0.0, 0.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
            Vector3d::new(-10.0, 0.0, 0.0) * si::N,
            100.0 * si::KG,
            Shape::Sphere(Sphere { radius }),
        ),
    );

    let e3 = Entity::new(
        "C",
        State::new(
            Vector3d::new(20.0, 0.0, 0.0) * si::M,
            Vector3d::new(0.0, 0.0, 0.0) * si::MPS,
            Vector3d::new(10.0, 0.0, 0.0) * si::N,
            100.0 * si::KG,
            Shape::Sphere(Sphere { radius }),
        ),
    );

    let sim = Simulation {
        simulation_time: 100.0 * si::S,
        time_step: 0.01 * si::S,
        system: System::new(
            vec![e1, e2, e3],
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
