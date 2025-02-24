use crate::models::spacecraft::SpacecraftProperties;
use crate::models::State;
use crate::numerics::quaternion::{compute_quaternion_derivative, Quaternion};
use nalgebra as na;

pub fn calculate_torque<T: SpacecraftProperties>(state: &State<T>) -> na::Vector3<f64> {
    // Gravity gradient torque
    let r = state.position;
    let r_mag = r.magnitude();
    let r_unit = r.normalize();

    let inertia = &state.inertia_tensor;

    // Transform body frame to inertial frame
    let rot_matrix = state.quaternion.to_rotation_matrix();
    let z_body = rot_matrix.transpose() * r_unit;

    // Calculate gravity gradient torque
    (3.0 * crate::constants::G * crate::constants::M_EARTH / (2.0 * r_mag.powi(3)))
        * z_body.cross(&(inertia * z_body))
}

pub fn angular_acceleration<T: SpacecraftProperties>(
    state: &State<T>,
    external_torque: Option<na::Vector3<f64>>,
) -> na::Vector3<f64> {
    let inertia = state.inertia_tensor;
    let w = state.angular_velocity;

    let torque = external_torque.unwrap_or_else(|| calculate_torque(state));
    let gyro = w.cross(&(inertia * w));

    inertia.try_inverse().unwrap() * (torque - gyro)
}

pub fn quaternion_derivative<T: SpacecraftProperties>(state: &State<T>) -> Quaternion {
    // Only use body angular velocity for quaternion propagation
    compute_quaternion_derivative(&state.quaternion, &state.angular_velocity)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{config::spacecraft::SimpleSat, constants::R_EARTH};
    use approx::assert_abs_diff_eq;
    use hifitime::Epoch;
    use test_case::test_case;

    #[test_case(
        State {
            position:na::Vector3::new(0.0,0.0,0.0),
            velocity:na::Vector3::new(0.0,0.0,0.0),
            quaternion:Quaternion::new(1.0,0.0,0.0,0.0),
            angular_velocity:na::Vector3::new(0.0,0.0,0.0),
            mass:1.0,inertia_tensor:na::Matrix3::identity(),
            spacecraft: &SimpleSat,
            epoch: Epoch::default(),
            mission_elapsed_time: 0.0,
            fuel_mass: 0.,
        },
        na::Vector3::new(0.0, 0.0, 0.0) =>
        ignore; // TODO: NaN result
        "zero torque"
    )]
    #[test_case(
        State {
            position: na::Vector3::new(6.471e6, 0.0, 0.0),
            velocity: na::Vector3::new(0.0, 7.848e3, 0.0),
            quaternion: Quaternion::new(1.0, 0.0, 0.0, 0.0),
            angular_velocity: na::Vector3::new(0.0, 0.0, 0.0),
            mass: 1.0,
            inertia_tensor: na::Matrix3::identity(),
            spacecraft: &SimpleSat,
            epoch: Epoch::default(),
            mission_elapsed_time: 0.0,
            fuel_mass: 0.0,
        },
        na::Vector3::new(0.0, 0.0, 0.0); // Expected no gravity gradient torque
        "100 km without any gravity gradient torque"
    )]
    #[test_case(
        State {
            position: na::Vector3::new(R_EARTH + 100.*1e3, 0.0, 0.0),
            velocity: na::Vector3::new(0.0, 7.848e3, 0.0),
            quaternion: Quaternion::new(0.99, 0.1, 0.05, 0.0),
            angular_velocity: na::Vector3::new(0.0, 0.0, 0.0),
            mass: 1.0,
            inertia_tensor: na::Matrix3::new( // Asymmetric inertia tensor
                10.0, 0.0, 0.0,
                0.0, 20.0, 0.0,
                0.0, 0.0, 30.0,
            ),
            spacecraft: &SimpleSat,
            epoch: Epoch::default(),
            mission_elapsed_time: 0.0,
            fuel_mass: 0.0,
        },
        na::Vector3::new(2.18e-8, -4.35e-6, 2.20e-7);
        "100 km orbit with gravity gradient torque"
    )]
    fn test_calculate_torque(state: State<SimpleSat>, expected: na::Vector3<f64>) {
        let result = calculate_torque(&state);
        assert_abs_diff_eq!(result, expected, epsilon = 1e-2);
    }

    #[test_case(
        State {
            position:na::Vector3::new(0.0,0.0,0.0),
            velocity:na::Vector3::new(0.0,0.0,0.0),
            quaternion:Quaternion::new(1.0,0.0,0.0,0.0),
            angular_velocity:na::Vector3::new(0.0,0.0,0.0),
            mass:1.0,inertia_tensor:na::Matrix3::identity(),
            spacecraft: &SimpleSat,
            epoch: Epoch::default(),
            mission_elapsed_time: 0.0,
            fuel_mass: 0.,
        },
        na::Vector3::new(0.0, 0.0, 0.0) => ignore; // TODO: NaN in result
        "zero angular acceleration"
    )]
    #[test_case(
        State {
            position: na::Vector3::new(6.471e6, 0.0, 0.0),
            velocity: na::Vector3::new(0.0, 7.848e3, 0.0),
            quaternion: Quaternion::new(1.0, 0.0, 0.0, 0.0),
            angular_velocity: na::Vector3::new(0.0, 0.0, 0.0),
            mass: 1.0,
            inertia_tensor: na::Matrix3::identity(),
            spacecraft: &SimpleSat,
            epoch: Epoch::default(),
            mission_elapsed_time: 0.0,
            fuel_mass: 0.0,
        },
        na::Vector3::new(0.0, 0.0, 0.0); // Expected zero acceleration
        "100 km orbit with no gravity gradient torque"
    )]
    #[test_case(
        State {
            position: na::Vector3::new(R_EARTH + 100.*1e3, 0.0, 0.0),
            velocity: na::Vector3::new(0.0, 7.848e3, 0.0),
            quaternion: Quaternion::new(0.99, 0.1, 0.05, 0.0),
            angular_velocity: na::Vector3::new(0.0, 0.0, 0.0),
            mass: 1.0,
            inertia_tensor: na::Matrix3::new( // Asymmetric inertia tensor
                10.0, 0.0, 0.0,
                0.0, 20.0, 0.0,
                0.0, 0.0, 30.0,
            ),
            spacecraft: &SimpleSat,
            epoch: Epoch::default(),
            mission_elapsed_time: 0.0,
            fuel_mass: 0.0,
        },
        na::Vector3::new(2.18e-9, -2.17e-7, 7.34e-9); // Expected angular acceleration
        "100 km orbit with gravity gradient angular acceleration"
    )]
    fn test_angular_acceleration(state: State<SimpleSat>, expected: na::Vector3<f64>) {
        let result = angular_acceleration(&state, None);
        assert_abs_diff_eq!(result, expected, epsilon = 1e-2);
    }
}
