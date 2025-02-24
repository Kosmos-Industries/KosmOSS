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
    let force_magnitude: f64 =
        -0.5 * spacecraft.drag_coefficient() * spacecraft.reference_area() * rho * v_po.powi(2);
    velocity.normalize() * force_magnitude
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{config::spacecraft::SimpleSat, constants::R_EARTH};

    use approx::assert_abs_diff_eq;
    use nalgebra as na;
    use test_case::test_case;

    // TODO: drag_force returns NaN because Environment::new(position).density returns NaN
    #[test_case(
        SimpleSat,
        na::Vector3::new(0.0, 0.0, 0.0),
        na::Vector3::new(0.0, 0.0, 0.0),
        na::Vector3::new(0.0, 0.0, 0.0) =>
        ignore; // TODO: NaN result
        "zero velocity"
    )]
    #[test_case(
        SimpleSat,
        na::Vector3::new(0.0, 0.0, 0.0),
        na::Vector3::new(1.0, 0.0, 0.0),
        na::Vector3::new(0.0, 0.0, 0.0) =>
        ignore; // TODO: NaN result
        "zero position"
    )]
    #[test_case(
        SimpleSat,
        na::Vector3::new(0.0, 0.0, R_EARTH + 500.*1e3),
        na::Vector3::new(0.0, 0.0, 1.0),
        na::Vector3::new(0.0, 0.0, 0.0);
        "high altitude"
    )]
    #[test_case(
        SimpleSat,
        na::Vector3::new(R_EARTH + 100.*1e3, 0.0, 0.0),
        na::Vector3::new(0.0, 7848., 0.0),  // Velocity: 7.66 km/s tangential to orbit
        na::Vector3::new(0.0, -242.28, 0.0);  // Expected drag force (placeholder value)
        "100km altitude"
    )]
    fn drag_force<T: SpacecraftProperties>(
        spacecraft: T,
        position: na::Vector3<f64>,
        velocity: na::Vector3<f64>,
        expected: na::Vector3<f64>,
    ) {
        let force = super::drag_force(&spacecraft, &position, &velocity);
        assert_abs_diff_eq!(force, expected, epsilon = 1e-2);
    }
}
