use super::attitude::{angular_acceleration, quaternion_derivative};
use super::drag::drag_force;
use super::gravity::gravity_acceleration;
use crate::models::State;
use crate::models::spacecraft::SpacecraftProperties;
use nalgebra as na;
use std::marker::PhantomData;

pub trait EquationsOfMotion {
    type State;
    fn compute_derivative(&self, state: &Self::State) -> Self::State;
}

pub struct SpacecraftDynamics<'a, T: SpacecraftProperties> {
    thrust: Option<na::Vector3<f64>>,
    torque: Option<na::Vector3<f64>>,
    _phantom: PhantomData<&'a T>,
}

impl<'a, T: SpacecraftProperties> SpacecraftDynamics<'a, T> {
    pub fn new(thrust: Option<na::Vector3<f64>>, torque: Option<na::Vector3<f64>>) -> Self {
        Self { 
            thrust, 
            torque,
            _phantom: PhantomData,
        }
    }
}

impl<'a, T: SpacecraftProperties> EquationsOfMotion for SpacecraftDynamics<'a, T> {
    type State = State<'a, T>;

    fn compute_derivative(&self, state: &Self::State) -> Self::State {
        let mut derivative = State::zero(state.spacecraft);

        // Position derivative is velocity
        derivative.position = state.velocity;

        // Velocity derivative (gravity + thrust + drag)
        derivative.velocity = gravity_acceleration(&state.position)
            + drag_force(state.spacecraft, &state.position, &state.velocity) / state.mass;
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
