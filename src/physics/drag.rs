use super::environment::Environment;
use crate::models::spacecraft::SpacecraftProperties;
use nalgebra as na;

pub fn drag_force<T: SpacecraftProperties>(
    spacecraft: &T,
    position: &na::Vector3<f64>,
    velocity: &na::Vector3<f64>,
) -> na::Vector3<f64> {
    let v_po: f64 = velocity.magnitude();
    let rho: f64 = Environment::new(position).density;

    let force_magnitude: f64 = -0.5 * spacecraft.drag_coefficient() * spacecraft.reference_area() * rho * v_po.powi(2);
    velocity.normalize() * force_magnitude
}
