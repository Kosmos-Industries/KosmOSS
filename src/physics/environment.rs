use nalgebra as na;
use crate::constants::*;

pub struct Environment {
    pub altitude: f64,
    pub density: f64,
    pub magnetic_field: na::Vector3<f64>,
    pub solar_flux: f64,
}

impl Environment {
    pub fn new(position: &na::Vector3<f64>) -> Self {
        let altitude = position.magnitude() - R_EARTH;
        
        // Simple exponential atmospheric model
        let scale_height = 7200.0; // meters
        let density = 1.225 * (-altitude / scale_height).exp();
        
        // Simplified dipole magnetic field model
        let r = position.magnitude();
        let m = 7.94e22; // Earth's magnetic dipole moment
        let b0 = (M_0 * m) / (4.0 * std::f64::consts::PI * r.powi(3));
        let magnetic_field = na::Vector3::new(0.0, 0.0, 2.0 * b0);
        
        Environment {
            altitude,
            density,
            magnetic_field,
            solar_flux: 1361.0, // W/m^2 at 1 AU
        }
    }
} 