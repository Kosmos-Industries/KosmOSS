use crate::constants::{G, M_EARTH};
use nalgebra as na;

pub fn gravity_acceleration(position: &na::Vector3<f64>) -> na::Vector3<f64> {
    let r: f64 = position.magnitude();
    let acceleration_magnitude: f64 = -G * M_EARTH / (r * r);
    position.normalize() * acceleration_magnitude
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use nalgebra as na;
    use test_case::test_case;

    #[test_case(
        na::Vector3::new(0.0, 0.0, 0.0),
        na::Vector3::new(0.0, 0.0, 0.0) => ignore; // TODO: NaN result
        "zero position vector (edge case)"
    )]
    #[test_case(
        na::Vector3::new(6.371e6, 0.0, 0.0), // Earth's surface
        na::Vector3::new(-9.81, 0.0, 0.0); // Expected acceleration
        "gravity at Earth's surface"
    )]
    #[test_case(
        na::Vector3::new(6.471e6, 0.0, 0.0), // 100 km altitude
        na::Vector3::new(-9.515, 0.0, 0.0); // Expected acceleration
        "gravity at 100 km altitude"
    )]
    #[test_case(
        na::Vector3::new(6.871e6, 0.0, 0.0), // 500 km altitude
        na::Vector3::new(-8.44, 0.0, 0.0); // Expected acceleration
        "gravity at 500 km altitude"
    )]
    fn test_gravity_acceleration(position: na::Vector3<f64>, expected: na::Vector3<f64>) {
        let result = gravity_acceleration(&position);
        assert_abs_diff_eq!(result, expected, epsilon = 1e-2);
    }
}
