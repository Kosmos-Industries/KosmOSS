use crate::numerics::quaternion::Quaternion;
use nalgebra as na;

pub struct GeometricAttitudeController {
    kp: f64,
    kd: f64,
    inertia: na::Matrix3<f64>,
}

impl GeometricAttitudeController {
    pub fn new(kp: f64, kd: f64, inertia: na::Matrix3<f64>) -> Self {
        Self { kp, kd, inertia }
    }

    pub fn compute_control_torque(
        &self,
        r_gcrs: &na::Vector3<f64>,
        v_gcrs: &na::Vector3<f64>,
        q_gcrs2body: &Quaternion,
        w_body: &na::Vector3<f64>,
    ) -> na::Vector3<f64> {
        // Get desired RSW frame
        let r_unit = r_gcrs.normalize();
        let h = r_gcrs.cross(v_gcrs);
        let w_unit = h.normalize();
        let s_unit = w_unit.cross(&r_unit);

        let r_gcrs2rsw = na::Matrix3::from_columns(&[r_unit, s_unit, w_unit]);

        // Current rotation matrix
        let r_current = q_gcrs2body.to_rotation_matrix();

        // Compute attitude error in SO(3)
        let r_error = r_current.transpose() * r_gcrs2rsw;
        let e = (r_error.transpose() - r_error) * 0.5;

        // Extract vector form of error
        let e_r = na::Vector3::new(e[(2, 1)], e[(0, 2)], e[(1, 0)]);

        // Compute desired angular velocity
        let orbital_rate = v_gcrs.magnitude() / r_gcrs.magnitude();
        let w_desired =
            r_current.transpose() * r_gcrs2rsw * na::Vector3::new(0.0, 0.0, -orbital_rate);

        // Angular velocity error
        let e_w = w_body - w_desired;

        // Geometric control law on SO(3)
        let mut control_torque = self.inertia * (-self.kp * e_r - self.kd * e_w);

        // Smooth saturation function
        let max_torque = 1.0; // N⋅m
        let torque_mag = control_torque.magnitude();

        if torque_mag > max_torque {
            let scale = max_torque * (1.0 - (-torque_mag / max_torque).exp()) / torque_mag;
            control_torque *= scale;
        }

        control_torque
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::PI;

    #[test]
    #[ignore = "TODO: FIX"]
    fn test_zero_error_case() {
        let inertia = na::Matrix3::identity();
        let controller = GeometricAttitudeController::new(1.0, 0.1, inertia);

        let r = na::Vector3::new(7000.0e3, 0.0, 0.0);
        let v = na::Vector3::new(0.0, 7.8e3, 0.0);
        let q = Quaternion::new(1.0, 0.0, 0.0, 0.0);
        let w = na::Vector3::zeros();

        let torque = controller.compute_control_torque(&r, &v, &q, &w);

        // Should be near zero for perfect alignment
        assert_relative_eq!(torque.magnitude(), 0.0, epsilon = 1e-10);
    }

    #[test]
    #[ignore = "TODO: FIX"]
    fn test_90_degree_error() {
        let inertia = na::Matrix3::identity();
        let controller = GeometricAttitudeController::new(1.0, 0.1, inertia);

        let r = na::Vector3::new(7000.0e3, 0.0, 0.0);
        let v = na::Vector3::new(0.0, 7.8e3, 0.0);

        // 90-degree rotation about Z
        let q = Quaternion::new((PI / 4.0).cos(), 0.0, 0.0, (PI / 4.0).sin());
        let w = na::Vector3::zeros();

        let torque = controller.compute_control_torque(&r, &v, &q, &w);

        // Should produce non-zero torque for misalignment
        assert!(torque.magnitude() > 0.0);
        // Should not exceed maximum torque
        assert!(torque.magnitude() <= 0.001);
    }
}
