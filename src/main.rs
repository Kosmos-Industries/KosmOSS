mod config;
mod constants;
mod coordinates;
mod gnc;
mod integrators;
mod models;
mod numerics;
mod physics;
use crate::numerics::quaternion::Quaternion;
use config::spacecraft::SimpleSat;
use constants::*;
use csv::Writer;
use gnc::control::attitude_controller::GeometricAttitudeController;
use gnc::guidance::hohmann::{ApsisTargeting, ApsisType};
use hifitime::{Duration, Epoch};
use integrators::rk4::RK4;
use models::State;
use nalgebra as na;
use physics::dynamics::SpacecraftDynamics;
use physics::energy::{calculate_angular_momentum, calculate_energy};
use physics::orbital::OrbitalMechanics;
use std::error::Error;
use std::fs::{self, File};
use std::path::Path;

fn main() -> Result<(), Box<dyn Error>> {
    let perigee_alt = 400_000.0; // meters
    let apogee_alt = 600_000.0; // meters
    let ra = WGS84_A + apogee_alt;
    let rp = WGS84_A + perigee_alt;
    let a = (ra + rp) / 2.0;
    let e = (ra - rp) / (ra + rp);

    let elements = na::Vector6::new(
        a,                     // semi-major axis
        e,                     // eccentricity
        51.6_f64.to_radians(), // inclination (ISS-like)
        0.0,                   // RAAN
        0.0,                   // argument of periapsis
        0.0,                   // true anomaly (starting at perigee)
    );

    let (initial_position, initial_velocity) = OrbitalMechanics::keplerian_to_cartesian(&elements);
    let orbital_period = OrbitalMechanics::compute_orbital_period(elements[0]);

    // Set simulation start and end times using proper time scales
    let start_time = Epoch::from_gregorian_utc(2024, 3, 15, 0, 0, 0, 0);
    let simulation_duration = Duration::from_seconds(orbital_period * 3.0);
    let _end_time = start_time + simulation_duration;

    // Create initial state with epoch
    let initial_state = State::new(
        SimpleSat::MASS,
        SimpleSat::inertia_tensor(),
        initial_position,
        initial_velocity,
        Quaternion::new(1.0, 0.0, 0.0, 0.0),
        na::Vector3::new(0.01, 0.0, 0.0),
        start_time,
    );

    let dt = 0.10; // Much smaller time step for accurate integration
    let simulation_time = orbital_period * 3.0;
    let steps = (simulation_time / dt) as usize;

    let mut state = initial_state;
    let initial_energy = calculate_energy(&state);
    let initial_angular_momentum = calculate_angular_momentum(&state);

    // Create output directory if it doesn't exist
    let output_dir = Path::new("output");
    fs::create_dir_all(output_dir)?;

    // Create CSV writer
    let file = File::create(output_dir.join("simulation_data.csv"))?;
    let mut writer = Writer::from_writer(file);
    
    // Modify CSV header to include UTC time
    writer.write_record(&[
        "UTC Time",
        "Time (s)",  // Changed from "Mission Elapsed Time (s)" for viz compatibility
        "Position X (km)",
        "Position Y (km)",
        "Position Z (km)",
        "Longitude (deg)",
        "Latitude (deg)",
        "Altitude (km)",
        "Quaternion W",
        "Quaternion X",
        "Quaternion Y",
        "Quaternion Z",
        "Angular Velocity X (rad/s)",
        "Angular Velocity Y (rad/s)",
        "Angular Velocity Z (rad/s)",
        "Energy Error",
        "Angular Momentum Error",
        "Control Torque X (N⋅m)",
        "Control Torque Y (N⋅m)",
        "Control Torque Z (N⋅m)",
        "Thrust X (N)",
        "Thrust Y (N)",
        "Thrust Z (N)",
    ])?;

    // Initialize controllers
    let attitude_controller = GeometricAttitudeController::new(
        1.0, // kp - proportional gain
        0.1, // kd - derivative gain
        SimpleSat::inertia_tensor(),
    );

    // Create Hohmann transfer guidance for raising apogee with 1 orbit delay
    let target_apogee = 800_000.0; // meters
    let hohmann_guidance = ApsisTargeting::new(
        WGS84_A + target_apogee,
        ApsisType::Apogee,
        orbital_period, // Start after one orbit
    );

    for i in 0..steps {
        let current_time = i as f64 * dt;
        let current_epoch = start_time + Duration::from_seconds(current_time);
        
        // Update state's time properties
        state.mission_elapsed_time = current_time;
        state.epoch = current_epoch;

        // Compute control inputs
        let thrust =
            hohmann_guidance.get_desired_force(&state.position, &state.velocity, current_time);

        let control_torque = attitude_controller.compute_control_torque(
            &state.position,
            &state.velocity,
            &state.quaternion,
            &state.angular_velocity,
        );

        // Update dynamics with control inputs
        let dynamics = SpacecraftDynamics::new(Some(thrust), Some(control_torque));
        let integrator = RK4::new(dynamics);

        // Calculate Earth rotation
        let gmst = (EARTH_ANGULAR_VELOCITY * current_time) % (2.0 * PI);

        // Add EOPData
        let eop = coordinates::coordinate_transformation::EOPData::from_epoch(current_epoch)
            .unwrap_or_else(|_| coordinates::coordinate_transformation::EOPData {
                x_pole: 0.161556, // Default values in arcseconds
                y_pole: 0.247219,
                ut1_utc: -0.0890529, // Default UT1-UTC offset in seconds
                lod: 0.0017,         // Length of day offset in seconds
                ddpsi: -0.052,       // Nutation corrections in arcseconds
                ddeps: -0.003,
            });

        // Convert to geographic coordinates
        let itrs_pos =
            crate::coordinates::coordinate_transformation::eci_to_itrs(&state.position, gmst, &eop);
        let (longitude, latitude, altitude) =
            crate::coordinates::coordinate_transformation::itrs_to_geodetic(&itrs_pos);

        // Write data to CSV if:
        // 1. It's a regular sampling interval (every 600 steps)
        // 2. OR there's a non-zero thrust being applied
        if i % 600 == 0 || thrust.magnitude() > 0.0 {
            let current_energy = calculate_energy(&state);
            let current_angular_momentum = calculate_angular_momentum(&state);

            let energy_error = (current_energy - initial_energy).abs() / initial_energy.abs();
            let angular_momentum_error = (current_angular_momentum - initial_angular_momentum)
                .magnitude()
                / initial_angular_momentum.magnitude();

            // Write data to CSV with attitude state
            writer.write_record(&[
                format!("{}", current_epoch),
                format!("{:.1}", current_time),
                format!("{:.3}", state.position[0] / 1000.0),
                format!("{:.3}", state.position[1] / 1000.0),
                format!("{:.3}", state.position[2] / 1000.0),
                format!("{:.6}", longitude),
                format!("{:.6}", latitude),
                format!("{:.3}", altitude / 1000.0), // Convert to km
                format!("{:.6}", state.quaternion.scalar()),
                format!("{:.6}", state.quaternion.vector()[0]),
                format!("{:.6}", state.quaternion.vector()[1]),
                format!("{:.6}", state.quaternion.vector()[2]),
                format!("{:.6}", state.angular_velocity[0]),
                format!("{:.6}", state.angular_velocity[1]),
                format!("{:.6}", state.angular_velocity[2]),
                format!("{:.6e}", energy_error),
                format!("{:.6e}", angular_momentum_error),
                format!("{:.6e}", control_torque[0]),
                format!("{:.6e}", control_torque[1]),
                format!("{:.6e}", control_torque[2]),
                format!("{:.6e}", thrust[0]),
                format!("{:.6e}", thrust[1]),
                format!("{:.6e}", thrust[2]),
            ])?;
        }
        state = integrator.integrate(&state, dt);
    }

    writer.flush()?;
    println!("Simulation data has been written to output/simulation_data.csv");

    Ok(())
}
