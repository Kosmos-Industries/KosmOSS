use crate::constants::{G, M_EARTH, PI};
use nalgebra as na;

pub struct OrbitalMechanics;

#[allow(non_snake_case)]
#[allow(dead_code)]
impl OrbitalMechanics {
    /// Converts Cartesian state (position and velocity) to Keplerian orbital elements
    /// Returns: [a, e, i, Omega, omega, nu]
    ///   a: semi-major axis [m]
    ///   e: eccentricity [-]
    ///   i: inclination [rad]
    ///   Omega: right ascension of ascending node [rad]
    ///   omega: argument of periapsis [rad]
    ///   nu: true anomaly [rad]
    pub fn cartesian_to_keplerian(r: &na::Vector3<f64>, v: &na::Vector3<f64>) -> na::Vector6<f64> {
        let mu = G * M_EARTH;
        let mut elements = na::Vector6::zeros();

        // Calculate angular momentum vector
        let h = r.cross(v);
        let h_mag = h.magnitude();

        // Calculate node vector
        let k = na::Vector3::new(0.0, 0.0, 1.0);
        let n = k.cross(&h);
        let n_mag = n.magnitude();

        // Calculate eccentricity vector
        let r_mag = r.magnitude();
        let v_mag = v.magnitude();
        let e_vec = ((v_mag * v_mag - mu / r_mag) * r - r.dot(v) * v) / mu;
        let e = e_vec.magnitude();
        elements[1] = e;

        // Semi-major axis
        let specific_energy = v_mag * v_mag / 2.0 - mu / r_mag;
        elements[0] = -mu / (2.0 * specific_energy);

        // Inclination
        elements[2] = (h.z / h_mag).acos();

        // Right ascension of ascending node
        elements[3] = if n_mag < 1e-11 {
            0.0
        } else {
            let mut raan = n.y.atan2(n.x);
            if raan < 0.0 {
                raan += 2.0 * PI;
            }
            raan
        };

        // Argument of periapsis
        elements[4] = if e < 1e-11 {
            0.0
        } else if n_mag < 1e-11 {
            let mut omega = e_vec.y.atan2(e_vec.x);
            if omega < 0.0 {
                omega += 2.0 * PI;
            }
            omega
        } else {
            let mut omega = (h.dot(&e_vec.cross(&n))).atan2(n.dot(&e_vec));
            if omega < 0.0 {
                omega += 2.0 * PI;
            }
            omega
        };

        // True anomaly
        elements[5] = if e < 1e-11 {
            if n_mag < 1e-11 {
                r.y.atan2(r.x)
            } else {
                n.dot(&r.cross(&n)).atan2(n.dot(r))
            }
        } else {
            let mut nu = h.dot(&e_vec.cross(r)).atan2(e_vec.dot(r));
            if nu < 0.0 {
                nu += 2.0 * PI;
            }
            nu
        };

        elements
    }

    pub fn compute_orbital_period(a: f64) -> f64 {
        2.0 * PI * (a.powi(3) / (G * M_EARTH)).sqrt()
    }

    pub fn compute_circular_velocity(r: f64) -> f64 {
        ((G * M_EARTH) / r).sqrt()
    }

    pub fn compute_apsides(r: &na::Vector3<f64>, v: &na::Vector3<f64>) -> (f64, f64) {
        let mu = G * M_EARTH;
        let r_mag = r.magnitude();
        let v_mag = v.magnitude();
        let specific_energy = (v_mag * v_mag / 2.0) - mu / r_mag;
        let h = r.cross(v);
        let h_mag2 = h.dot(&h);

        let a = -mu / (2.0 * specific_energy);
        let e = (1.0 + (2.0 * specific_energy * h_mag2) / (mu * mu)).sqrt();

        let ra = a * (1.0 + e);
        let rp = a * (1.0 - e);

        (ra, rp)
    }

    pub fn is_near_apsis(
        r: &na::Vector3<f64>,
        v: &na::Vector3<f64>,
        tolerance: f64,
    ) -> (bool, bool) {
        let (ra, rp) = Self::compute_apsides(r, v);
        let r_mag = r.magnitude();

        let at_apogee = (r_mag - ra).abs() < tolerance;
        let at_perigee = (r_mag - rp).abs() < tolerance;

        (at_apogee, at_perigee)
    }

    // Anomaly conversion functions
    pub fn true_to_eccentric_anomaly(nu: f64, e: f64) -> f64 {
        if e < 1e-11 {
            return nu;
        }

        let cos_nu = nu.cos();
        let mut E = ((1.0 - e * e).sqrt() * nu.sin()).atan2(e + cos_nu);

        if E < 0.0 {
            E += 2.0 * PI;
        }
        E
    }

    pub fn eccentric_to_mean_anomaly(E: f64, e: f64) -> f64 {
        let mut M = E - e * E.sin();
        if M < 0.0 {
            M += 2.0 * PI;
        }
        M
    }

    #[allow(dead_code)]
    pub fn mean_to_eccentric_anomaly(M: f64, e: f64, tolerance: f64, max_iterations: i32) -> f64 {
        if e < 1e-11 {
            return M;
        }

        // Initial guess
        let mut E = if M < PI { M + e / 2.0 } else { M - e / 2.0 };

        // Newton-Raphson iteration
        for _ in 0..max_iterations {
            let delta = (E - e * E.sin() - M) / (1.0 - e * E.cos());
            E -= delta;
            if delta.abs() <= tolerance {
                break;
            }
        }

        if E < 0.0 {
            E += 2.0 * PI;
        }
        E
    }

    /// Converts Keplerian orbital elements to Cartesian state vectors
    /// Input elements: [a, e, i, Omega, omega, nu]
    ///   a: semi-major axis [m]
    ///   e: eccentricity [-]
    ///   i: inclination [rad]
    ///   Omega: right ascension of ascending node [rad]
    ///   omega: argument of periapsis [rad]
    ///   nu: true anomaly [rad]
    /// Returns: (position, velocity) in ECI frame [m, m/s]
    pub fn keplerian_to_cartesian(
        elements: &na::Vector6<f64>,
    ) -> (na::Vector3<f64>, na::Vector3<f64>) {
        let mu = G * M_EARTH;
        let (a, e, i, omega_cap, omega, nu) = (
            elements[0],
            elements[1],
            elements[2],
            elements[3],
            elements[4],
            elements[5],
        );

        // Calculate position and velocity in orbital plane
        let p = a * (1.0 - e * e);
        let r_mag = p / (1.0 + e * nu.cos());

        // Position in orbital plane
        let r_orbital = na::Vector3::new(r_mag * nu.cos(), r_mag * nu.sin(), 0.0);

        // Velocity in orbital plane
        let v_orbital = na::Vector3::new(
            -(mu / p).sqrt() * nu.sin(),
            (mu / p).sqrt() * (e + nu.cos()),
            0.0,
        );

        // Rotation matrices
        let rot_omega = na::Rotation3::from_axis_angle(&na::Vector3::z_axis(), omega);
        let rot_i = na::Rotation3::from_axis_angle(&na::Vector3::x_axis(), i);
        let rot_omega_cap = na::Rotation3::from_axis_angle(&na::Vector3::z_axis(), omega_cap);

        // Transform to ECI frame
        let transform = rot_omega_cap * rot_i * rot_omega;
        let r_eci = transform * r_orbital;
        let v_eci = transform * v_orbital;

        (r_eci, v_eci)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use nalgebra as na;
    use test_case::test_case;

    #[test_case(
        na::Vector3::new(0.0, 0.0, 0.0),
        na::Vector3::new(0.0, 0.0, 0.0),
        na::Vector6::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) => ignore; // TODO: NaNs in result
        "zero position and velocity"
    )]
    fn cartesian_to_keplerian(r: na::Vector3<f64>, v: na::Vector3<f64>, result: na::Vector6<f64>) {
        let elements = super::OrbitalMechanics::cartesian_to_keplerian(&r, &v);
        assert_abs_diff_eq!(elements, result, epsilon = 1e-2);
    }

    fn compute_orbital_period(input: f64, expected: f64) {
        let result = super::OrbitalMechanics::compute_orbital_period(input);
        assert_abs_diff_eq!(result, expected, epsilon = 1e-2);
    }

    fn compute_circular_velocity(r: f64, expected: f64) {
        let result = super::OrbitalMechanics::compute_circular_velocity(r);
        assert_abs_diff_eq!(result, expected, epsilon = 1e-2);
    }

    fn compute_apsides(r: na::Vector3<f64>, v: na::Vector3<f64>, expected: (f64, f64)) {
        let result = super::OrbitalMechanics::compute_apsides(&r, &v);
        assert_abs_diff_eq!(result.0, expected.0, epsilon = 1e-2);
        assert_abs_diff_eq!(result.1, expected.1, epsilon = 1e-2);
    }

    fn is_near_apsis(
        r: na::Vector3<f64>,
        v: na::Vector3<f64>,
        tolerance: f64,
        expected: (bool, bool),
    ) {
        let result = super::OrbitalMechanics::is_near_apsis(&r, &v, tolerance);
        assert_eq!(result, expected);
    }

    fn true_to_eccentric_anomaly(nu: f64, e: f64, expected: f64) {
        let result = super::OrbitalMechanics::true_to_eccentric_anomaly(nu, e);
        assert_abs_diff_eq!(result, expected, epsilon = 1e-2);
    }

    fn eccentric_to_mean_anomaly(E: f64, e: f64, expected: f64) {
        let result = super::OrbitalMechanics::eccentric_to_mean_anomaly(E, e);
        assert_abs_diff_eq!(result, expected, epsilon = 1e-2);
    }

    fn mean_to_eccentric_anomaly(
        M: f64,
        e: f64,
        tolerance: f64,
        max_iterations: i32,
        expected: f64,
    ) {
        let result =
            super::OrbitalMechanics::mean_to_eccentric_anomaly(M, e, tolerance, max_iterations);
        assert_abs_diff_eq!(result, expected, epsilon = 1e-2);
    }

    fn keplerian_to_cartesian(
        elements: na::Vector6<f64>,
        result: (na::Vector3<f64>, na::Vector3<f64>),
    ) {
        let (r, v) = super::OrbitalMechanics::keplerian_to_cartesian(&elements);
        assert_abs_diff_eq!(r, result.0, epsilon = 1e-2);
        assert_abs_diff_eq!(v, result.1, epsilon = 1e-2);
    }
}
