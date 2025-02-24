use nalgebra as na;

#[cfg(test)]
use approx::AbsDiffEq;

/// Quaternion utilities for spacecraft attitude dynamics
/// Following scalar-first convention: q = [q0; q1; q2; q3] = [w; x; y; z]
#[derive(Debug, Clone, PartialEq)]
pub struct Quaternion {
    pub data: na::Vector4<f64>,
}

#[cfg(test)]
impl AbsDiffEq for Quaternion {
    type Epsilon = f64;

    fn default_epsilon() -> Self::Epsilon {
        f64::default_epsilon()
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        self.data.abs_diff_eq(&other.data, epsilon)
    }
}

impl Quaternion {
    pub fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Quaternion {
            data: na::Vector4::new(w, x, y, z),
        }
    }

    pub fn scalar(&self) -> f64 {
        self.data[0]
    }

    pub fn vector(&self) -> na::Vector3<f64> {
        na::Vector3::new(self.data[1], self.data[2], self.data[3])
    }

    #[allow(dead_code)]
    pub fn normalize(&self) -> Self {
        Quaternion {
            data: self.data.normalize(),
        }
    }

    pub fn to_rotation_matrix(&self) -> na::Matrix3<f64> {
        let q0 = self.data[0];
        let q1 = self.data[1];
        let q2 = self.data[2];
        let q3 = self.data[3];

        na::Matrix3::new(
            1.0 - 2.0 * (q2 * q2 + q3 * q3),
            2.0 * (q1 * q2 - q0 * q3),
            2.0 * (q1 * q3 + q0 * q2),
            2.0 * (q1 * q2 + q0 * q3),
            1.0 - 2.0 * (q1 * q1 + q3 * q3),
            2.0 * (q2 * q3 - q0 * q1),
            2.0 * (q1 * q3 - q0 * q2),
            2.0 * (q2 * q3 + q0 * q1),
            1.0 - 2.0 * (q1 * q1 + q2 * q2),
        )
    }

    #[allow(dead_code)]
    pub fn multiply(&self, other: &Quaternion) -> Self {
        let q1 = self;
        let q2 = other;

        Quaternion::new(
            q1.scalar() * q2.scalar() - q1.vector().dot(&q2.vector()),
            q1.scalar() * q2.vector()[0]
                + q2.scalar() * q1.vector()[0]
                + (q1.vector()[1] * q2.vector()[2] - q1.vector()[2] * q2.vector()[1]),
            q1.scalar() * q2.vector()[1]
                + q2.scalar() * q1.vector()[1]
                + (q1.vector()[2] * q2.vector()[0] - q1.vector()[0] * q2.vector()[2]),
            q1.scalar() * q2.vector()[2]
                + q2.scalar() * q1.vector()[2]
                + (q1.vector()[0] * q2.vector()[1] - q1.vector()[1] * q2.vector()[0]),
        )
        .normalize()
    }
}

pub fn compute_quaternion_derivative(q: &Quaternion, w: &na::Vector3<f64>) -> Quaternion {
    let wx = w[0];
    let wy = w[1];
    let wz = w[2];

    Quaternion::new(
        -0.5 * (q.data[1] * wx + q.data[2] * wy + q.data[3] * wz),
        0.5 * (q.data[0] * wx + q.data[2] * wz - q.data[3] * wy),
        0.5 * (q.data[0] * wy + q.data[3] * wx - q.data[1] * wz),
        0.5 * (q.data[0] * wz + q.data[1] * wy - q.data[2] * wx),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use test_case::test_case;

    /// Test quaternion to rotation matrix conversion
    #[test_case(Quaternion::new(1.0, 0.0, 0.0, 0.0), na::Matrix3::identity(); "identity quaternion")]
    #[test_case(Quaternion::new(0.0, 1.0, 0.0, 0.0), na::Matrix3::new(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0); "x-axis rotation")]
    #[test_case(Quaternion::new(0.0, 0.0, 1.0, 0.0), na::Matrix3::new(-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0); "y-axis rotation")]
    #[test_case(Quaternion::new(0.0, 0.0, 0.0, 1.0), na::Matrix3::new(-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0); "z-axis rotation")]
    #[test_case(
        Quaternion::new(0.92388, 0.2706, 0.2706, 0.0),
        na::Matrix3::new(
            0.85356,  0.14645,  0.50000,
            0.14645,  0.85356, -0.50000,
           -0.50000,  0.50000,  0.70716
        );
        "45-degree rotation about XY-plane"
    )]
    fn quaternion_to_rotation_matrix(q: Quaternion, expected: na::Matrix3<f64>) {
        assert_abs_diff_eq!(q.to_rotation_matrix(), expected, epsilon = 1e-2);
    }

    /// Test quaternion multiplication
    #[test_case(Quaternion::new(1.0, 0.0, 0.0, 0.0), Quaternion::new(1.0, 0.0, 0.0, 0.0), Quaternion::new(1.0, 0.0, 0.0, 0.0); "identity quaternion")]
    #[test_case(Quaternion::new(1.0, 0.0, 0.0, 0.0), Quaternion::new(0.0, 1.0, 0.0, 0.0), Quaternion::new(0.0, 1.0, 0.0, 0.0); "multiply by identity quaternion")]
    #[test_case(Quaternion::new(0.0, 1.0, 0.0, 0.0), Quaternion::new(0.0, 1.0, 0.0, 0.0), Quaternion::new(-1.0, 0.0, 0.0, 0.0); "multiply by itself")]
    #[test_case(Quaternion::new(0.0, 1.0, 0.0, 0.0), Quaternion::new(0.0, 0.0, 1.0, 0.0), Quaternion::new(0.0, 0.0, 0.0, 1.0); "multiply x and y axes")]
    #[test_case(
        Quaternion::new(0.7071, 0.7071, 0.0, 0.0),
        Quaternion::new(0.7071, 0.0, 0.7071, 0.0),
        Quaternion::new(0.5, 0.5, 0.5, 0.5);
        "90-degree rotations about x and y axes"
    )]
    fn quaternion_multiplication(q1: Quaternion, q2: Quaternion, expected: Quaternion) {
        assert_abs_diff_eq!(q1.multiply(&q2), expected, epsilon = 1e-2);
    }

    /// Test quaternion derivative computation
    #[test_case(Quaternion::new(1.0, 0.0, 0.0, 0.0), na::Vector3::zeros(), Quaternion::new(0.0, 0.0, 0.0, 0.0); "zero angular velocity")]
    #[test_case(Quaternion::new(1.0, 0.0, 0.0, 0.0), na::Vector3::new(1.0, 0.0, 0.0), Quaternion::new(0.0, 0.5, 0.0, 0.0); "x-axis rotation")]
    #[test_case(Quaternion::new(1.0, 0.0, 0.0, 0.0), na::Vector3::new(0.0, 1.0, 0.0), Quaternion::new(0.0, 0.0, 0.5, 0.0); "y-axis rotation")]
    #[test_case(Quaternion::new(1.0, 0.0, 0.0, 0.0), na::Vector3::new(0.0, 0.0, 1.0), Quaternion::new(0.0, 0.0, 0.0, 0.5); "z-axis rotation")]
    #[test_case(
        Quaternion::new(0.7071, 0.0, 0.0, 0.7071),
        na::Vector3::new(0.0, 0.0, 1.0),
        Quaternion::new(-0.35355, 0.0, 0.0, 0.35355);
        "90-degree rotation about z with 1 rad/s"
    )]
    fn quaternion_derivative(q: Quaternion, w: na::Vector3<f64>, expected: Quaternion) {
        assert_abs_diff_eq!(
            compute_quaternion_derivative(&q, &w),
            expected,
            epsilon = 1e-2
        );
    }
}
