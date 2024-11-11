use dimensioned::{si, Sqrt};
use csv;

use physical_machine::physics::system::System;
use physical_machine::physics::state::shape::Shape;
use physical_machine::utils::identity::Identity;
use physical_machine::utils::round::Round;

use super::plotter::{DataSeries, Plotter};

pub struct Simulation {
    pub simulation_time: si::Second<f64>,
    pub time_step: si::Second<f64>,
    pub system: System,
}

#[derive(Clone)]
pub struct SimulationStep {
    pub time: si::Second<f64>,
    pub system: System,
}

impl Simulation {
    fn step(&mut self, check_preserved_quantities: bool) {
        let digits = 8;
        let momentum_before = self
            .system
            .get_momentum()
            .norm2()
            .sqrt()
            .value_unsafe
            .round_digits(digits);
        let energy_before = self.system.get_energy().value_unsafe.round_digits(digits);
        self.system.next_state(self.time_step);
        let momentum_after = self
            .system
            .get_momentum()
            .norm2()
            .sqrt()
            .value_unsafe
            .round_digits(digits);
        let energy_after = self.system.get_energy().value_unsafe.round_digits(digits);
        let allowed_error: f64 = 0.007;
        if check_preserved_quantities && momentum_before != 0.0 && energy_before != 0.0 {
            let momentum_diff = 1.0 - momentum_after / momentum_before;
            println!("{}% difference in momentum.", momentum_diff);
            assert!(
                momentum_after >= momentum_before * (1.0 - allowed_error)
                    && momentum_after <= momentum_before * (1.0 + allowed_error),
                "Momentum does not match: {} -> {} ({}% difference) at {}",
                momentum_before,
                momentum_after,
                momentum_diff,
                self.system.get_current_time(),
            );
            let energy_diff = 1.0 - energy_after / energy_before;
            println!("{}% difference in energy.", energy_diff);
            assert!(
                energy_after >= energy_before * (1.0 - allowed_error)
                    && energy_after <= energy_before * (1.0 + allowed_error),
                "Energy does not match: {} -> {} ({}% difference) at {}",
                energy_before,
                energy_after,
                energy_diff,
                self.system.get_current_time(),
            );
        }
    }

    pub fn run(mut self, check_preserved_quantities: bool) -> Vec<SimulationStep> {
        let mut time = 0.0 * si::S;
        let mut history = Vec::<SimulationStep>::new();
        history.push(SimulationStep {
            time,
            system: self.system.clone(),
        });
        while time <= self.simulation_time {
            self.step(check_preserved_quantities);
            time = time + self.time_step;
            history.push(SimulationStep {
                time,
                system: self.system.clone(),
            });
        }
        history
    }
}

pub fn write_csv(history: &Vec<SimulationStep>, filename: &str) {
    let file = std::fs::OpenOptions::new()
        .write(true)
        .create(true)
        .append(false)
        .truncate(true)
        .open(filename)
        .unwrap();
    let mut writer = csv::Writer::from_writer(file);
    let mut header = Vec::<String>::new();
    header.push("time".to_string());
    for (idx, _entity) in history[0].system.get_entities().iter().enumerate() {
        header.push("e".to_string() + &idx.to_string() + "_identity");
        header.push("e".to_string() + &idx.to_string() + "_location_x");
        header.push("e".to_string() + &idx.to_string() + "_location_y");
        header.push("e".to_string() + &idx.to_string() + "_location_z");
        header.push("e".to_string() + &idx.to_string() + "_velocity_x");
        header.push("e".to_string() + &idx.to_string() + "_velocity_y");
        header.push("e".to_string() + &idx.to_string() + "_velocity_z");
        header.push("e".to_string() + &idx.to_string() + "_net_force_x");
        header.push("e".to_string() + &idx.to_string() + "_net_force_y");
        header.push("e".to_string() + &idx.to_string() + "_net_force_z");
        header.push("e".to_string() + &idx.to_string() + "mass");
        header.push("e".to_string() + &idx.to_string() + "radius");
    }
    match writer.write_record(&header) {
        Ok(()) => (),
        Err(e) => panic!("{}", e),
    };
    for sim_step in history {
        let mut entry = Vec::<String>::new();
        entry.push(sim_step.time.value_unsafe.to_string());
        for entity in sim_step.system.get_entities() {
            entry.push(entity.get_identity().to_string());
            entry.push(entity.get_state().get_location().x.value_unsafe.to_string());
            entry.push(entity.get_state().get_location().y.value_unsafe.to_string());
            entry.push(entity.get_state().get_location().z.value_unsafe.to_string());
            entry.push(entity.get_state().get_velocity().x.value_unsafe.to_string());
            entry.push(entity.get_state().get_velocity().y.value_unsafe.to_string());
            entry.push(entity.get_state().get_velocity().z.value_unsafe.to_string());
            entry.push(entity.get_state().get_net_force().x.value_unsafe.to_string());
            entry.push(entity.get_state().get_net_force().y.value_unsafe.to_string());
            entry.push(entity.get_state().get_net_force().z.value_unsafe.to_string());
            entry.push(entity.get_state().get_mass().value_unsafe.to_string());
            match entity.get_state().get_shape() {
                Shape::Sphere(s) => entry.push(s.radius.value_unsafe.to_string()),
                _ => panic!("Unknown shape."),
            };
        }
        match writer.write_record(&entry) {
            Ok(()) => (),
            Err(e) => panic!("{}", e),
        };
    }
    match writer.flush() {
        Ok(()) => (),
        Err(e) => panic!("{}", e),
    };
}

pub fn plot_results(history: &Vec<SimulationStep>, filename: &str, title: &str) {
    let mut data_series = Vec::<DataSeries>::new();
    for entity in history[0].system.get_entities() {
        data_series.push(DataSeries {
            name: entity.get_identity().to_string(),
            data: Vec::<(f64, f64)>::new(),
        });
    }
    for (i, _) in history[0].system.get_entities().iter().enumerate() {
        for data_point in history {
            let entity = data_point.system.get_entities()[i].clone();
            data_series[i].data.push((
                data_point.time.value_unsafe,
                entity.get_state().get_location().x.value_unsafe,
            ));
        }
    }
    data_series.plot(filename, title);
}
