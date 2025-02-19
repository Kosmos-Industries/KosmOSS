use config::spacecraft::SimpleSat;
use csv::Writer;
use hifitime::{Duration, Epoch};
use kosmoss::config;
use kosmoss::constants::*;
use kosmoss::coordinates;
use kosmoss::fsm::state_machine::SpacecraftFSM;
use kosmoss::gnc::control::attitude_controller::GeometricAttitudeController;
use kosmoss::gnc::guidance::hohmann::{ApsisTargeting, ApsisType};
use kosmoss::integrators::rk4::RK4;
use kosmoss::models::State;
use kosmoss::numerics::quaternion::Quaternion;
use kosmoss::physics::dynamics::SpacecraftDynamics;
use kosmoss::physics::energy::{calculate_angular_momentum, calculate_energy};
use kosmoss::physics::orbital::OrbitalMechanics;
use nalgebra as na;
use std::fs::{self, File};
use std::path::Path;

// Integration test for an entire spacecraft simulation
// TODO: This test should be broken into smaller unit and integration tests
// to test individual components of the simulation
#[test]
fn integration_test() -> Result<(), Box<dyn std::error::Error>> {
    static SPACECRAFT: SimpleSat = SimpleSat;
    let perigee_alt = 50_000.0; // meters
    let apogee_alt = 400_000.0; // meters
    let ra = WGS84_A + apogee_alt;
    let rp = WGS84_A + perigee_alt;
    let a = (ra + rp) / 2.0;
    let e = (ra - rp) / (ra + rp);

    let elements = na::Vector6::new(
        a,                     // semi-major axis
        e,                     // eccentricity
        89.0_f64.to_radians(), // inclination (ISS-like)
        PI * 1.0,              // RAAN
        PI * 0.075,            // argument of periapsis
        PI,                    // true anomaly (starting at perigee)
    );

    let (initial_position, initial_velocity) = OrbitalMechanics::keplerian_to_cartesian(&elements);
    //let orbital_period = OrbitalMechanics::compute_orbital_period(elements[0]);

    // Set simulation start and end times using proper time scales
    let start_time = Epoch::from_gregorian_utc(2024, 3, 1, 23, 10, 0, 0);
    let simulation_duration = Duration::from_seconds(3200.0);
    let _end_time = start_time + simulation_duration;

    // Create initial state with epoch
    let initial_state = State::new(
        &SPACECRAFT,
        SimpleSat::inertia_tensor(),
        initial_position,
        initial_velocity,
        Quaternion::new(1.0, 0.0, 0.0, 0.0),
        na::Vector3::new(0.05, 0.02, 0.01), // Higher initial angular velocity
        start_time,
    );

    let dt = 0.01; // Much smaller time step for accurate integration
    let simulation_time = 3200.0;
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
        "Time (s)",
        "Position X (km)",
        "Position Y (km)",
        "Position Z (km)",
        "Velocity X (km/s)",
        "Velocity Y (km/s)",
        "Velocity Z (km/s)",
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
        "Current State",
        "Time Since State Change (s)",
    ])?;

    // Initialize controllers
    let attitude_controller = GeometricAttitudeController::new(
        1.0, // kp - proportional gain
        0.1, // kd - derivative gain
        SimpleSat::inertia_tensor(),
    );

    // Create Hohmann transfer guidance for raising apogee with 1 orbit delay
    let target_apogee = 400_000.0; // meters
    let hohmann_guidance = ApsisTargeting::new(
        WGS84_A + target_apogee,
        ApsisType::Apogee,
        0.0, // Start after one orbit
    );

    // Create FSM (Finite State Machine)
    let mut fsm = SpacecraftFSM::new();

    // Schedule a maneuver at t=1000s
    let maneuver_time = 1000.0;
    let mut maneuver_scheduled = false;

    for i in 0..steps {
        let current_time = i as f64 * dt;
        let current_epoch = start_time + Duration::from_seconds(current_time);

        // Update state's time properties
        state.mission_elapsed_time = current_time;
        state.epoch = current_epoch;

        // Evaluate state transitions
        fsm.evaluate_transition(&state);

        // Check if it's time to start the maneuver
        if !maneuver_scheduled && current_time >= maneuver_time {
            if fsm.command_maneuver(current_time) {
                println!("Maneuver commanded at t={:.2}s", current_time);
                maneuver_scheduled = true;
            }
        }

        // Compute control inputs based on current state
        let (thrust, control_torque) = if fsm.should_apply_control() {
            let control_torque = attitude_controller.compute_control_torque(
                &state.position,
                &state.velocity,
                &state.quaternion,
                &state.angular_velocity,
            );

            let thrust = if fsm.should_apply_thrust() {
                hohmann_guidance.get_desired_force(
                    &SPACECRAFT,
                    &state.position,
                    &state.velocity,
                    current_time,
                )
            } else {
                na::Vector3::zeros()
            };

            (thrust, control_torque)
        } else {
            (na::Vector3::zeros(), na::Vector3::zeros())
        };

        // Update dynamics with control inputs
        let dynamics = SpacecraftDynamics::<SimpleSat>::new(Some(thrust), Some(control_torque));
        let integrator = RK4::new(dynamics);

        // Add EOPData
        let eop = coordinates::coordinate_transformation::EOPData::from_epoch(current_epoch)
            .unwrap_or_else(|e| {
                eprintln!("Warning: Failed to get EOP data: {}. Using defaults.", e);
                coordinates::coordinate_transformation::EOPData {
                    x_pole: 0.161556,
                    y_pole: 0.247219,
                    ut1_utc: -0.0890529,
                    lod: 0.0017,
                    ddpsi: -0.052,
                    ddeps: -0.003,
                }
            });

        // Convert to geographic coordinates
        let itrs_pos = crate::coordinates::coordinate_transformation::gcrs_to_itrs(
            &state.position,
            &current_epoch,
            &eop,
        );
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
                &current_epoch.to_string(),
                &current_time.to_string(),
                &(state.position.x / 1000.0).to_string(),
                &(state.position.y / 1000.0).to_string(),
                &(state.position.z / 1000.0).to_string(),
                &(state.velocity.x / 1000.0).to_string(),
                &(state.velocity.y / 1000.0).to_string(),
                &(state.velocity.z / 1000.0).to_string(),
                &longitude.to_string(),
                &latitude.to_string(),
                &(altitude / 1000.0).to_string(), // Convert to km
                &state.quaternion.scalar().to_string(),
                &state.quaternion.vector()[0].to_string(),
                &state.quaternion.vector()[1].to_string(),
                &state.quaternion.vector()[2].to_string(),
                &state.angular_velocity[0].to_string(),
                &state.angular_velocity[1].to_string(),
                &state.angular_velocity[2].to_string(),
                &energy_error.to_string(),
                &angular_momentum_error.to_string(),
                &control_torque[0].to_string(),
                &control_torque[1].to_string(),
                &control_torque[2].to_string(),
                &thrust[0].to_string(),
                &thrust[1].to_string(),
                &thrust[2].to_string(),
                &fsm.get_current_state().to_string(),
                &(current_time - fsm.get_last_state_change()).to_string(),
            ])?;
        }
        state = integrator.integrate(&state, dt);
    }

    writer.flush()?;
    println!("Simulation data has been written to output/simulation_data.csv");

    Ok(())
}
