use super::environment::Environment;
use crate::config::spacecraft::SimpleSat;
use crate::constants::PI;

use nalgebra as na;

pub fn drag_force(position: &na::Vector3<f64>, velocity: &na::Vector3<f64>) -> na::Vector3<f64> {
    let v_po: f64 = velocity.magnitude();

    let a_cs: f64 = PI * SimpleSat::R_SPACECRAFT.powi(2);
    let c_d: f64 = SimpleSat::C_D;
    let rho: f64 = Environment::new(&position).density;

    let force_magnitude: f64 = -0.5 * c_d * a_cs * rho * v_po.powi(2);
    velocity.normalize() * force_magnitude
}
