use crate::constants::{G, M_EARTH};
use crate::models::state::State;
use crate::models::spacecraft::SpacecraftProperties;
use nalgebra as na;

pub fn calculate_energy<T: SpacecraftProperties>(state: &State<T>) -> f64 {
    let r = state.position.magnitude();
    let v = state.velocity.magnitude();

    let kinetic = 0.5 * state.mass * v * v;
    let potential = -G * M_EARTH * state.mass / r;

    kinetic + potential
}

pub fn calculate_angular_momentum<T: SpacecraftProperties>(state: &State<T>) -> na::Vector3<f64> {
    state.position.cross(&(state.velocity * state.mass))
}
