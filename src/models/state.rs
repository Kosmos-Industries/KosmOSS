use crate::numerics::quaternion::Quaternion;
use hifitime::Epoch;
use nalgebra as na;
use crate::models::spacecraft::SpacecraftProperties;

#[derive(Debug)]
pub struct State<'a, T: SpacecraftProperties> {
    pub spacecraft: &'a T,
    pub mass: f64,
    pub inertia_tensor: na::Matrix3<f64>,

    // Orbital state
    pub position: na::Vector3<f64>,
    pub velocity: na::Vector3<f64>,

    // Attitude state
    pub quaternion: Quaternion,
    pub angular_velocity: na::Vector3<f64>,

    // Time properties
    pub epoch: Epoch,
    pub mission_elapsed_time: f64,
    pub fuel_mass: f64,
}

impl<'a, T: SpacecraftProperties> State<'a, T> {
    pub fn new(
        spacecraft: &'a T,
        inertia: na::Matrix3<f64>,
        position: na::Vector3<f64>,
        velocity: na::Vector3<f64>,
        quaternion: Quaternion,
        angular_velocity: na::Vector3<f64>,
        epoch: Epoch,
    ) -> Self {
        let mass = spacecraft.mass();
        State {
            spacecraft,
            mass,
            inertia_tensor: inertia,
            position,
            velocity,
            quaternion,
            angular_velocity,
            epoch,
            mission_elapsed_time: 0.0,
            fuel_mass: mass * 0.1, // 10% of total mass is fuel
        }
    }

    pub fn zero(spacecraft: &'a T) -> Self {
        State {
            spacecraft,
            mass: spacecraft.mass(),
            inertia_tensor: na::Matrix3::zeros(),
            position: na::Vector3::zeros(),
            velocity: na::Vector3::zeros(),
            quaternion: Quaternion::new(1.0, 0.0, 0.0, 0.0),
            angular_velocity: na::Vector3::zeros(),
            epoch: Epoch::now().expect("Failed to get current time"),
            mission_elapsed_time: 0.0,
            fuel_mass: 0.0,
        }
    }
}

impl<'a, T: SpacecraftProperties> std::ops::Add for State<'a, T> {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        State {
            spacecraft: self.spacecraft,
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
            epoch: self.epoch,
            mission_elapsed_time: self.mission_elapsed_time + other.mission_elapsed_time,
            fuel_mass: self.fuel_mass,
        }
    }
}

impl<'a, T: SpacecraftProperties> std::ops::Mul<f64> for State<'a, T> {
    type Output = Self;

    fn mul(self, scalar: f64) -> Self {
        State {
            spacecraft: self.spacecraft,
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
            epoch: self.epoch,
            mission_elapsed_time: self.mission_elapsed_time * scalar,
            fuel_mass: self.fuel_mass,
        }
    }
}

impl<'a, T: SpacecraftProperties> Clone for State<'a, T> {
    fn clone(&self) -> Self {
        State {
            spacecraft: self.spacecraft,
            mass: self.mass,
            inertia_tensor: self.inertia_tensor,
            position: self.position,
            velocity: self.velocity,
            quaternion: self.quaternion.clone(),
            angular_velocity: self.angular_velocity,
            epoch: self.epoch,
            mission_elapsed_time: self.mission_elapsed_time,
            fuel_mass: self.fuel_mass,
        }
    }
}
