use nalgebra as na;
use crate::constants::{G, M_EARTH};

pub fn gravity_acceleration(position: &na::Vector3<f64>) -> na::Vector3<f64> {
    let r = position.magnitude();
    let acceleration_magnitude = -G * M_EARTH / (r * r);
    position.normalize() * acceleration_magnitude
} 