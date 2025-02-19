use super::eop_errors::EOPErrors;
use crate::constants::*;
use crate::coordinates::eop_manager::EOPManager;
use hifitime::Epoch;
use lazy_static::lazy_static;
use nalgebra as na;
use std::sync::{Mutex, OnceLock};

lazy_static! {
    static ref EOP_MANAGER: Mutex<EOPManager> = Mutex::new(EOPManager::new());
    static ref INIT_STATUS: OnceLock<bool> = OnceLock::new(); // Tracks if `initialize()` was called
}

#[derive(Clone)]
pub struct EOPData {
    pub x_pole: f64,  // Polar motion x (arcsec)
    pub y_pole: f64,  // Polar motion y (arcsec)
    pub ut1_utc: f64, // UT1-UTC difference (seconds)
    pub lod: f64,     // Length of day offset (seconds)
    pub ddpsi: f64,   // Nutation correction to longitude (arcsec)
    pub ddeps: f64,   // Nutation correction to obliquity (arcsec)
}

impl TryFrom<Epoch> for EOPData {
    type Error = EOPErrors;

    /// Try to get EOP data for a given epoch
    /// This will fetch the EOP data from the cache file if available otherwise it will fail
    fn try_from(epoch: Epoch) -> Result<Self, Self::Error> {
        let mut manager = EOP_MANAGER.lock().unwrap();

        // Ensure `initialize()` is only called once
        if INIT_STATUS.get().is_none() {
            manager.initialize()?;
            INIT_STATUS.set(true).unwrap();
        }

        // Fetch EOP data
        manager.get_eop_data(epoch, false)
    }
}

impl EOPData {
    /// Interpolate EOP data between two epochs
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

    /// Refresh the EOP data cache
    /// This will fetch the latest data from the Celestrak website and store it in the cache
    /// file
    pub fn refresh_data() -> Result<(), EOPErrors> {
        let mut manager = EOP_MANAGER.lock().unwrap();
        manager.refresh_data()
    }
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
    let b = a * (1.0 - f); // Semi-minor axis
    let e2 = 2.0 * f - f * f; // First eccentricity squared

    let p = (x * x + y * y).sqrt();

    // Handle special cases
    if p < 1e-10 {
        let longitude: f64 = 0.0;
        let latitude: f64 = if z < 0.0 { -PI / 2.0 } else { PI / 2.0 };
        let altitude: f64 = (z.abs() - b).max(0.0); // Ensure non-negative
        return (longitude.to_degrees(), latitude.to_degrees(), altitude);
    }

    // Initial guess
    let mut latitude = z.atan2(p * (1.0 - e2));

    // Iterative solution
    for _ in 0..5 {
        // Usually converges in 2-3 iterations
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
    let altitude = (p / latitude.cos() - n).max(0.0); // Ensure non-negative

    (longitude.to_degrees(), latitude.to_degrees(), altitude)
}

/// Convert GCRS to ITRS using IAU 2000/2006 CIO-based transformation
pub fn gcrs_to_itrs(position: &na::Vector3<f64>, epoch: &Epoch, eop: &EOPData) -> na::Vector3<f64> {
    // Convert arcseconds to radians
    let arcsec_to_rad = std::f64::consts::PI / (180.0 * 3600.0);

    // Get time since J2000.0 in Julian centuries
    let t = (epoch.to_jde_tai(hifitime::Unit::Day) - 2451545.0) / 36525.0;

    // Calculate Earth Rotation Angle (ERA)
    let ut1_jd = epoch.to_jde_tai(hifitime::Unit::Day) + (eop.ut1_utc / 86400.0);
    let theta = 2.0 * PI * (0.7790572732640 + 1.00273781191135448 * (ut1_jd - 2451545.0));

    // Get X, Y coordinates of the CIP in GCRS (simplified IAU 2006/2000A, accuracy ~1 mas)
    let x = -0.016617 + 2004.191898 * t - 0.4297829 * t * t - 0.19861834 * t * t * t;
    let y = -0.006951 - 0.025896 * t - 22.4072747 * t * t + 0.00190059 * t * t * t;
    let x = x * arcsec_to_rad;
    let y = y * arcsec_to_rad;

    // Calculate s = CIO locator (simplified, accuracy ~0.1 mas)
    let s = -0.0015506 + (-0.0001729 - 0.000000127 * t) * t;
    let s = s * arcsec_to_rad;

    // Form the celestial-to-intermediate matrix (Q)
    let d = 1.0 + 0.5 * (x * x + y * y);

    let q_matrix = na::Matrix3::new(
        1.0 - x * x / 2.0 / d,
        -x * y / 2.0 / d,
        -x / d,
        -x * y / 2.0 / d,
        1.0 - y * y / 2.0 / d,
        -y / d,
        x,
        y,
        1.0 / d,
    );

    // Form the Earth rotation matrix (R)
    let r_matrix = na::Rotation3::from_axis_angle(&na::Vector3::z_axis(), theta - s);

    // Polar motion matrix (W)
    let xp = eop.x_pole * arcsec_to_rad;
    let yp = eop.y_pole * arcsec_to_rad;
    let w_matrix = na::Rotation3::from_euler_angles(-yp, -xp, 0.0);

    // Combined transformation
    let transform = w_matrix.matrix() * r_matrix.matrix() * q_matrix;

    // Apply transformation
    transform * position
}
