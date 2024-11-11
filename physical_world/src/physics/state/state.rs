use dimensioned::{si, Sqrt};
use vector3d::Vector3d;

use crate::physics::state::shape::Shape;
use crate::physics::system::PRECISION;

#[derive(Clone, Debug)]
pub struct State {
    location: Vector3d<si::Meter<f64>>,
    velocity: Vector3d<si::MeterPerSecond<f64>>,
    net_force: Vector3d<si::Newton<f64>>,
    mass: si::Kilogram<f64>,
    shape: Shape,
}

impl State {
    pub fn new(
        location: Vector3d<si::Meter<f64>>,
        velocity: Vector3d<si::MeterPerSecond<f64>>,
        net_force: Vector3d<si::Newton<f64>>,
        mass: si::Kilogram<f64>,
        shape: Shape,
    ) -> State {
        State {
            location,
            velocity,
            net_force,
            mass,
            shape,
        }
    }

    pub fn get_location(&self) -> Vector3d<si::Meter<f64>> {
        self.location
    }

    pub fn get_velocity(&self) -> Vector3d<si::MeterPerSecond<f64>> {
        self.velocity
    }

    pub fn get_net_force(&self) -> Vector3d<si::Newton<f64>> {
        self.net_force
    }

    pub fn get_mass(&self) -> si::Kilogram<f64> {
        self.mass
    }

    pub fn get_shape(&self) -> Shape {
        self.shape
    }

    pub fn evolve(&mut self, elapsed_time: si::Second<f64>) {
        self.location = self.location + self.velocity * elapsed_time;
        self.velocity = self.velocity + (self.net_force / self.mass) * elapsed_time;
    }
}

impl std::ops::Add<&State> for &State {
    type Output = State;
    fn add(self, rhs: &State) -> State {
        State {
            location: self.location + rhs.get_location(),
            velocity: self.velocity + rhs.get_velocity(),
            net_force: self.net_force + rhs.get_net_force(),
            mass: self.mass + rhs.get_mass(),
            shape: self.shape + rhs.get_shape(),
        }
    }
}

impl std::ops::Sub<&State> for &State {
    type Output = State;
    fn sub(self, rhs: &State) -> State {
        State {
            location: self.location - rhs.get_location(),
            velocity: self.velocity - rhs.get_velocity(),
            net_force: self.net_force - rhs.get_net_force(),
            mass: self.mass - rhs.get_mass(),
            shape: self.shape - rhs.get_shape(),
        }
    }
}

impl std::fmt::Display for State {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "\t\tLocation: {}\n\t\tVelocity: {}\n\t\tNet force: {}\n\t\tMass: {}\n\t\tShape: {}\n",
            self.location, self.velocity, self.net_force, self.mass, self.shape
        )
    }
}

impl std::cmp::PartialEq for State {
    fn eq(&self, other: &Self) -> bool {
        let diff = State::new(
            (self.location - other.location) * PRECISION,
            (self.velocity - other.velocity) * PRECISION,
            (self.net_force - other.net_force) * PRECISION,
            (self.mass - other.mass) * PRECISION,
            Shape::None,
        );
        diff.location.norm2().sqrt() < 1.0 * si::M
            && diff.velocity.norm2().sqrt() < 1.0 * si::MPS
            && diff.net_force.norm2().sqrt() < 1.0 * si::N
            && diff.mass < 1.0 * si::KG
            && self.shape == other.shape
    }
}
