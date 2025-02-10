use crate::models::State;
use crate::numerics::quaternion::{compute_quaternion_derivative, Quaternion};
use nalgebra as na;

pub fn calculate_torque(state: &State) -> na::Vector3<f64> {
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

pub fn angular_acceleration(
    state: &State,
    external_torque: Option<na::Vector3<f64>>,
) -> na::Vector3<f64> {
    let inertia = state.inertia_tensor;
    let w = state.angular_velocity;

    let torque = external_torque.unwrap_or_else(|| calculate_torque(state));
    let gyro = w.cross(&(inertia * w));

    inertia.try_inverse().unwrap() * (torque - gyro)
}

pub fn quaternion_derivative(state: &State) -> Quaternion {
    // Only use body angular velocity for quaternion propagation
    compute_quaternion_derivative(&state.quaternion, &state.angular_velocity)
}
