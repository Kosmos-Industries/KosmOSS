use crate::constants::{G, M_EARTH};
use crate::models::spacecraft::SpacecraftProperties;
use crate::models::state::State;
use nalgebra as na;

pub fn calculate_energy<T: SpacecraftProperties>(state: &State<T>) -> f64 {
    let r = state.position.magnitude();
    let v = state.velocity.magnitude();

    let kinetic = 0.5 * state.mass * v * v;
    let potential = -G * M_EARTH * state.mass / r;

    kinetic + potential
}

pub fn calculate_angular_momentum<T: SpacecraftProperties>(state: &State<T>) -> na::Vector3<f64> {
    state.position.cross(&(state.velocity * state.mass))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        config::spacecraft::SimpleSat, constants::R_EARTH, numerics::quaternion::Quaternion,
    };
    use approx::assert_abs_diff_eq;
    use hifitime::Epoch;
    use test_case::test_case;

    #[test_case(
        State {
            position:na::Vector3::new(0.0,0.0,0.0),
            velocity:na::Vector3::new(0.0,0.0,0.0),
            quaternion:Quaternion::new(1.0,0.0,0.0,0.0),
            angular_velocity:na::Vector3::new(0.0,0.0,0.0),
            mass:1.0,inertia_tensor:na::Matrix3::identity(),
            spacecraft: &SimpleSat,
            epoch: Epoch::default(),
            mission_elapsed_time: 0.0,
            fuel_mass: 0.,
        },
        0.0 => ignore; // TODO: -inf result
        "zero energy"
    )]
    #[test_case(
        State {
            position:na::Vector3::new(R_EARTH,0.0, 0.0),
            velocity:na::Vector3::new(0.0,0.0,0.0),
            quaternion:Quaternion::new(1.0,0.0,0.0,0.0),
            angular_velocity:na::Vector3::new(0.0,0.0,0.0),
            mass:100.,
            inertia_tensor:na::Matrix3::identity(),
            spacecraft: &SimpleSat,
            epoch: Epoch::default(),
            mission_elapsed_time: 0.0,
            fuel_mass: 0.,
        },
        -6256305069.84;
        "energy at Earth's surface"
    )]
    fn test_energy<T: SpacecraftProperties>(state: State<T>, expected_energy: f64) {
        assert_abs_diff_eq!(calculate_energy(&state), expected_energy, epsilon = 1e-2);
    }

    #[test_case(
        State {
            position: na::Vector3::new(0.0, 0.0, 0.0),
            velocity: na::Vector3::new(0.0, 0.0, 0.0),
            quaternion: Quaternion::new(1.0, 0.0, 0.0, 0.0),
            angular_velocity: na::Vector3::new(0.0, 0.0, 0.0),
            mass: 1.0,
            inertia_tensor: na::Matrix3::identity(),
            spacecraft: &SimpleSat,
            epoch: Epoch::default(),
            mission_elapsed_time: 0.0,
            fuel_mass: 0.0,
        },
        na::Vector3::new(0.0, 0.0, 0.0); // Expected zero angular momentum
        "zero angular momentum"
    )]
    #[test_case(
        State {
            position: na::Vector3::new(R_EARTH + 100e3, 0.0, 0.0), // 100 km above Earth's surface
            velocity: na::Vector3::new(0.0, 7.848e3, 0.0), // Orbital velocity (7.848 km/s)
            quaternion: Quaternion::new(1.0, 0.0, 0.0, 0.0),
            angular_velocity: na::Vector3::new(0.0, 0.0, 0.0),
            mass: 100.0,
            inertia_tensor: na::Matrix3::identity(),
            spacecraft: &SimpleSat,
            epoch: Epoch::default(),
            mission_elapsed_time: 0.0,
            fuel_mass: 0.0,
        },
        na::Vector3::new(0.0, 0.0, 5.0784408e12);
        "100 km orbit angular momentum"
    )]
    fn test_angular_momentum<T: SpacecraftProperties>(
        state: State<T>,
        expected_momentum: na::Vector3<f64>,
    ) {
        let result = calculate_angular_momentum(&state);
        assert_abs_diff_eq!(result, expected_momentum, epsilon = 1e6);
    }
}
