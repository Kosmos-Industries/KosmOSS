use crate::numerics::quaternion::Quaternion;
use nalgebra as na;

#[allow(dead_code)] // TODO: Remove this once we have a proper quaternion implementation.
#[derive(Debug, Clone)]
pub struct State {
    // Mass properties
    pub mass: f64,
    pub inertia_tensor: na::Matrix3<f64>,

    // Orbital state
    pub position: na::Vector3<f64>,
    pub velocity: na::Vector3<f64>,

    // Attitude state
    pub quaternion: Quaternion,
    pub angular_velocity: na::Vector3<f64>,

    // Additional properties
    pub time: f64,
    pub fuel_mass: f64,
}

impl State {
    pub fn new(
        mass: f64,
        inertia: na::Matrix3<f64>,
        position: na::Vector3<f64>,
        velocity: na::Vector3<f64>,
        quaternion: Quaternion,
        angular_velocity: na::Vector3<f64>,
    ) -> Self {
        State {
            mass,
            inertia_tensor: inertia,
            position,
            velocity,
            quaternion,
            angular_velocity,
            time: 0.0,
            fuel_mass: mass * 0.1, // 10% of total mass is fuel
        }
    }

    pub fn zero() -> Self {
        State {
            mass: 0.0,
            inertia_tensor: na::Matrix3::zeros(),
            position: na::Vector3::zeros(),
            velocity: na::Vector3::zeros(),
            quaternion: Quaternion::new(1.0, 0.0, 0.0, 0.0),
            angular_velocity: na::Vector3::zeros(),
            time: 0.0,
            fuel_mass: 0.0,
        }
    }
}

impl std::ops::Add for State {
    type Output = State;

    fn add(self, other: State) -> State {
        State {
            mass: self.mass,
            inertia_tensor: self.inertia_tensor,
            position: self.position + other.position,
            velocity: self.velocity + other.velocity,
            quaternion: Quaternion::new(
                self.quaternion.scalar() + other.quaternion.scalar(),
                self.quaternion.vector()[0] + other.quaternion.vector()[0],
                self.quaternion.vector()[1] + other.quaternion.vector()[1],
                self.quaternion.vector()[2] + other.quaternion.vector()[2],
            ),
            angular_velocity: self.angular_velocity + other.angular_velocity,
            time: self.time + other.time,
            fuel_mass: self.fuel_mass,
        }
    }
}

impl std::ops::Mul<f64> for State {
    type Output = State;

    fn mul(self, scalar: f64) -> State {
        State {
            mass: self.mass,
            inertia_tensor: self.inertia_tensor,
            position: self.position * scalar,
            velocity: self.velocity * scalar,
            quaternion: Quaternion::new(
                self.quaternion.scalar() * scalar,
                self.quaternion.vector()[0] * scalar,
                self.quaternion.vector()[1] * scalar,
                self.quaternion.vector()[2] * scalar,
            ),
            angular_velocity: self.angular_velocity * scalar,
            time: self.time * scalar,
            fuel_mass: self.fuel_mass,
        }
    }
}
