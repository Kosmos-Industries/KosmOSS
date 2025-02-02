use nalgebra as na;
use crate::constants::{G, M_EARTH};
use crate::models::State;

pub fn calculate_energy(state: &State) -> f64 {
    let r = state.position.magnitude();
    let v = state.velocity.magnitude();
    
    let kinetic = 0.5 * state.mass * v * v;
    let potential = -G * M_EARTH * state.mass / r;
    
    kinetic + potential
}

pub fn calculate_angular_momentum(state: &State) -> na::Vector3<f64> {
    state.position.cross(&(state.velocity * state.mass))
} 