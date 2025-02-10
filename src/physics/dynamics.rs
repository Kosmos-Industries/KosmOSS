use super::attitude::{angular_acceleration, quaternion_derivative};
use super::gravity::gravity_acceleration;
use crate::models::State;
use nalgebra as na;
pub trait EquationsOfMotion {
    type State;

    fn compute_derivative(&self, state: &Self::State) -> Self::State;
}

pub struct SpacecraftDynamics {
    thrust: Option<na::Vector3<f64>>,
    torque: Option<na::Vector3<f64>>,
}

impl SpacecraftDynamics {
    pub fn new(thrust: Option<na::Vector3<f64>>, torque: Option<na::Vector3<f64>>) -> Self {
        Self { thrust, torque }
    }
}

impl EquationsOfMotion for SpacecraftDynamics {
    type State = State;

    fn compute_derivative(&self, state: &State) -> State {
        let mut derivative = State::zero();

        // Position derivative is velocity
        derivative.position = state.velocity;

        // Velocity derivative (gravity + thrust)
        derivative.velocity = gravity_acceleration(&state.position);
        if let Some(thrust) = &self.thrust {
            derivative.velocity += thrust / state.mass;
        }

        // Angular acceleration (Euler's equation)
        derivative.angular_velocity = angular_acceleration(state, self.torque);

        // Quaternion derivative
        derivative.quaternion = quaternion_derivative(state);

        derivative
    }
}
