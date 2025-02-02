use nalgebra as na;

/// Quaternion utilities for spacecraft attitude dynamics
/// Following scalar-first convention: q = [q0; q1; q2; q3] = [w; x; y; z]
#[derive(Debug, Clone)]
pub struct Quaternion {
    pub data: na::Vector4<f64>
}

impl Quaternion {
    pub fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Quaternion {
            data: na::Vector4::new(w, x, y, z)
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
            data: self.data.normalize()
        }
    }

    pub fn to_rotation_matrix(&self) -> na::Matrix3<f64> {
        let q0 = self.data[0];
        let q1 = self.data[1];
        let q2 = self.data[2];
        let q3 = self.data[3];

        na::Matrix3::new(
            1.0-2.0*(q2*q2+q3*q3), 2.0*(q1*q2-q0*q3),     2.0*(q1*q3+q0*q2),
            2.0*(q1*q2+q0*q3),     1.0-2.0*(q1*q1+q3*q3), 2.0*(q2*q3-q0*q1),
            2.0*(q1*q3-q0*q2),     2.0*(q2*q3+q0*q1),     1.0-2.0*(q1*q1+q2*q2)
        )
    }
    
    #[allow(dead_code)]
    pub fn multiply(&self, other: &Quaternion) -> Self {
        let q1 = self;
        let q2 = other;
        
        Quaternion::new(
            q1.scalar() * q2.scalar() - q1.vector().dot(&q2.vector()),
            q1.scalar() * q2.vector()[0] + q2.scalar() * q1.vector()[0] + 
                (q1.vector()[1] * q2.vector()[2] - q1.vector()[2] * q2.vector()[1]),
            q1.scalar() * q2.vector()[1] + q2.scalar() * q1.vector()[1] + 
                (q1.vector()[2] * q2.vector()[0] - q1.vector()[0] * q2.vector()[2]),
            q1.scalar() * q2.vector()[2] + q2.scalar() * q1.vector()[2] + 
                (q1.vector()[0] * q2.vector()[1] - q1.vector()[1] * q2.vector()[0])
        ).normalize()
    }
}

pub fn compute_quaternion_derivative(q: &Quaternion, w: &na::Vector3<f64>) -> Quaternion {
    let wx = w[0];
    let wy = w[1];
    let wz = w[2];
    
    Quaternion::new(
        -0.5 * (q.data[1]*wx + q.data[2]*wy + q.data[3]*wz),
         0.5 * (q.data[0]*wx + q.data[2]*wz - q.data[3]*wy),
         0.5 * (q.data[0]*wy + q.data[3]*wx - q.data[1]*wz),
         0.5 * (q.data[0]*wz + q.data[1]*wy - q.data[2]*wx)
    )
} 