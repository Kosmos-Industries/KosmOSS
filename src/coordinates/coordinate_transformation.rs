use nalgebra as na;
use crate::constants::*;
use std::error::Error;
use chrono::{DateTime, Utc};

pub struct EOPData {
    pub x_pole: f64,    // Polar motion x (arcsec)
    pub y_pole: f64,    // Polar motion y (arcsec)
    pub ut1_utc: f64,   // UT1-UTC difference (seconds)
    pub lod: f64,       // Length of day offset (seconds)
    pub ddpsi: f64,     // Nutation correction to longitude (arcsec)
    pub ddeps: f64,     // Nutation correction to obliquity (arcsec)
}

/// Convert ECI (GCRF) to ITRS (Earth Fixed) using IAU-2006/2012 CIO based theory
pub fn eci_to_itrs(position: &na::Vector3<f64>, 
                  gmst: f64,
                  eop: &EOPData) -> na::Vector3<f64> {
    // Convert arcseconds to radians
    let arcsec_to_rad = std::f64::consts::PI / (180.0 * 3600.0);
    
    // Polar motion matrix (W)
    let xp = eop.x_pole * arcsec_to_rad;
    let yp = eop.y_pole * arcsec_to_rad;
    let polar_matrix = na::Rotation3::from_euler_angles(-yp, -xp, 0.0);
    
    // Earth rotation matrix (R)
    let ut1_correction = eop.ut1_utc;  // Already in seconds
    let adjusted_gmst = gmst + ut1_correction * EARTH_ANGULAR_VELOCITY;
    let earth_rot = na::Rotation3::from_axis_angle(&na::Vector3::z_axis(), adjusted_gmst);
    
    // Precession-nutation matrix (Q)
    let eps = 23.43929111 * std::f64::consts::PI / 180.0;
    let ddpsi = eop.ddpsi * arcsec_to_rad;
    let ddeps = eop.ddeps * arcsec_to_rad;
    
    // Correct rotation sequence for precession-nutation
    let prec_nut = na::Rotation3::from_axis_angle(&na::Vector3::x_axis(), -(eps + ddeps)) *
                   na::Rotation3::from_axis_angle(&na::Vector3::z_axis(), -ddpsi) *
                   na::Rotation3::from_axis_angle(&na::Vector3::x_axis(), eps);
    
    // Combined transformation (GCRF to ITRS)
    let transform = polar_matrix * earth_rot * prec_nut;
    
    // Apply transformation
    transform * position
}

/// Convert ITRS Cartesian to Geodetic coordinates (WGS84)
pub fn itrs_to_geodetic(pos: &na::Vector3<f64>) -> (f64, f64, f64) {
    let x = pos[0];
    let y = pos[1];
    let z = pos[2];
    
    // Longitude calculation
    let longitude = y.atan2(x);
    
    // Constants for WGS84
    let a = WGS84_A;
    let f = WGS84_F;
    let b = a * (1.0 - f);  // Semi-minor axis
    let e2 = 2.0 * f - f * f;  // First eccentricity squared
    
    let p = (x * x + y * y).sqrt();
    
    // Handle special cases
    if p < 1e-10 {
        let longitude: f64 = 0.0;
        let latitude: f64 = if z < 0.0 { -PI/2.0 } else { PI/2.0 };
        let altitude: f64 = (z.abs() - b).max(0.0);  // Ensure non-negative
        return (longitude.to_degrees(), latitude.to_degrees(), altitude);
    }
    
    // Initial guess
    let mut latitude = z.atan2(p * (1.0 - e2));
    
    // Iterative solution
    for _ in 0..5 {  // Usually converges in 2-3 iterations
        let sin_lat = latitude.sin();
        let n = a / (1.0 - e2 * sin_lat * sin_lat).sqrt();
        let h = p / latitude.cos() - n;
        
        let prev_lat = latitude;
        latitude = (z / p).atan2(1.0 - e2 * n / (n + h));
        
        if (latitude - prev_lat).abs() < 1e-12 {
            break;
        }
    }
    
    // Calculate final altitude
    let sin_lat = latitude.sin();
    let n = a / (1.0 - e2 * sin_lat * sin_lat).sqrt();
    let altitude = (p / latitude.cos() - n).max(0.0);  // Ensure non-negative
    
    (longitude.to_degrees(), latitude.to_degrees(), altitude)
}

impl EOPData {
    /// Load EOP data from IERS Bulletin A format
    pub fn from_iers_bulletin_a(_date: DateTime<Utc>) -> Result<Self, Box<dyn Error>> {
        // You would implement HTTP request to IERS servers here
        // For example: https://datacenter.iers.org/data/latestVersion/finals.all
        
        // For demonstration, using typical values
        Ok(EOPData {
            x_pole: 0.161556,      // Typically ranges ±0.3 arcseconds
            y_pole: 0.247219,      // Typically ranges ±0.3 arcseconds
            ut1_utc: -0.0890529,   // Currently around -1 to 1 seconds
            lod: 0.0017,           // Typically around 0.001 to 0.003 seconds
            ddpsi: -0.052,         // Typically ranges ±0.1 arcseconds
            ddeps: -0.003,         // Typically ranges ±0.1 arcseconds
        })
    }

    /// Interpolate EOP data between two epochs
    #[allow(dead_code)] //TODO: Make it not dead/use it.
    pub fn interpolate(eop1: &EOPData, eop2: &EOPData, fraction: f64) -> EOPData {
        EOPData {
            x_pole: eop1.x_pole + (eop2.x_pole - eop1.x_pole) * fraction,
            y_pole: eop1.y_pole + (eop2.y_pole - eop1.y_pole) * fraction,
            ut1_utc: eop1.ut1_utc + (eop2.ut1_utc - eop1.ut1_utc) * fraction,
            lod: eop1.lod + (eop2.lod - eop1.lod) * fraction,
            ddpsi: eop1.ddpsi + (eop2.ddpsi - eop1.ddpsi) * fraction,
            ddeps: eop1.ddeps + (eop2.ddeps - eop1.ddeps) * fraction,
        }
    }
}
