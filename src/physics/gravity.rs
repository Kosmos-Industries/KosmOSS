use crate::constants::{G, M_EARTH};
use nalgebra as na;

pub fn gravity_acceleration(position: &na::Vector3<f64>) -> na::Vector3<f64> {
    let r: f64 = position.magnitude();
    let acceleration_magnitude: f64 = -G * M_EARTH / (r * r);
    position.normalize() * acceleration_magnitude
}
