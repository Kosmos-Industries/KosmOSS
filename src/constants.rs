pub const G: f64 = 6.67430e-11; // Gravitational constant (m³/kg/s²)
pub const M_EARTH: f64 = 5.972e24; // Mass of Earth (kg)
#[allow(dead_code)]
pub const R_EARTH: f64 = 6.371e6; // Radius of Earth (m)
#[allow(dead_code)]
pub const ORBIT_ALTITUDE: f64 = 400_000.0; // Orbital altitude for LEO (m)

// Environmental constants
pub const M_0: f64 = 4.0 * std::f64::consts::PI * 1e-7; // Vacuum permeability
// pub const SOLAR_CONSTANT: f64 = 1361.0; // Solar constant at 1 AU (W/m^2)
// pub const EARTH_J2: f64 = 1.08263e-3; // Earth's J2 perturbation coefficient
pub const EARTH_ANGULAR_VELOCITY: f64 = 7.2921150e-5; // Earth's rotation rate (rad/s)
pub const WGS84_A: f64 = 6378137.0; // Semi-major axis [m]
pub const WGS84_F: f64 = 1.0 / 298.257223563; // Flattening

// // Spacecraft properties
// pub const C_D: f64 = 2.2;
// pub const R_SPACECRAFT: f64 = 1.0; // meters
// pub const REFLECTIVITY_COEFFICIENT: f64 = 0.3;

// Math
pub const PI: f64 = std::f64::consts::PI;
