use nalgebra as na;

pub struct SimpleSat;

impl SimpleSat {
    pub const MASS: f64 = 100.0; // kg
    pub const C_D: f64 = 2.2;
    pub const R_SPACECRAFT: f64 = 1.0; // meters

    pub fn inertia_tensor() -> na::Matrix3<f64> {
        na::Matrix3::new(10.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 10.0)
    }
}
