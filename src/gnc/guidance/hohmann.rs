use crate::config::spacecraft::SimpleSat;
use crate::constants::{G, M_EARTH};
use crate::physics::orbital::OrbitalMechanics;
use nalgebra as na;

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ApsisType {
    Perigee,
    Apogee,
}

#[allow(dead_code)]
pub struct ApsisTargeting {
    target_radius: f64,
    apsis_type: ApsisType,
    start_time: f64,
}

#[allow(dead_code)]
impl ApsisTargeting {
    pub fn new(target_radius: f64, apsis_type: ApsisType, start_time: f64) -> Self {
        Self {
            target_radius,
            apsis_type,
            start_time,
        }
    }

    // Getter methods
    pub fn get_apsis_type(&self) -> ApsisType {
        self.apsis_type
    }

    pub fn get_target_radius(&self) -> f64 {
        self.target_radius
    }

    pub fn get_desired_force(
        &self,
        r_current: &na::Vector3<f64>,
        v_current: &na::Vector3<f64>,
        time_since_start: f64,
    ) -> na::Vector3<f64> {
        if time_since_start < self.start_time {
            return na::Vector3::zeros();
        }

        // Get current apsides
        let (ra, rp) = OrbitalMechanics::compute_apsides(r_current, v_current);

        // Determine if we need to burn
        let should_burn = match self.apsis_type {
            ApsisType::Perigee => (rp - self.target_radius).abs() > 100.0, // 100m tolerance
            ApsisType::Apogee => (ra - self.target_radius).abs() > 100.0,  // 100m tolerance
        };

        if !should_burn {
            return na::Vector3::zeros();
        }

        // Check if we're at the correct apsis for burning
        let (at_apogee, at_perigee) = OrbitalMechanics::is_near_apsis(
            r_current, v_current, 100.0, // 100m tolerance
        );

        // At apogee, burn prograde to raise perigee
        // At perigee, burn prograde to raise apogee
        if (self.apsis_type == ApsisType::Perigee && at_apogee)
            || (self.apsis_type == ApsisType::Apogee && at_perigee)
        {
            let burn_direction = v_current.normalize();

            // Calculate required delta-v based on current orbit
            let r = r_current.magnitude();
            let v = v_current.magnitude();
            let mu = G * M_EARTH;

            // Calculate target velocity
            let target_v = match self.apsis_type {
                ApsisType::Apogee => {
                    // At perigee, calculate required velocity for new apogee
                    (mu * (2.0 / r - 2.0 / (self.target_radius + r))).sqrt()
                }
                ApsisType::Perigee => {
                    // At apogee, calculate required velocity for new perigee
                    (mu * (2.0 / r - 2.0 / (self.target_radius + r))).sqrt()
                }
            };

            let delta_v = target_v - v;
            let mut burn_magnitude = delta_v.abs().min(100.0); // Limit to 100 m/s per step
            if delta_v < 0.0 {
                burn_magnitude *= -1.0;
            }

            return burn_direction * burn_magnitude * SimpleSat::MASS;
        }

        na::Vector3::zeros()
    }
}
