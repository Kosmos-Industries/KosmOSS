use super::spacecraft_states::SpacecraftState;
use crate::models::spacecraft::SpacecraftProperties;
use crate::models::State as VehicleState;

pub struct SpacecraftFSM {
    current_state: SpacecraftState,
    angular_velocity_threshold: f64,
    emergency_angular_velocity: f64,
    last_state_change: f64,
    last_message_time: f64,
}

impl SpacecraftFSM {
    pub fn new() -> Self {
        Self {
            current_state: SpacecraftState::SafeMode,
            angular_velocity_threshold: 0.01,
            emergency_angular_velocity: 0.5,
            last_state_change: 0.0,
            last_message_time: -1.0,
        }
    }

    pub fn get_current_state(&self) -> SpacecraftState {
        self.current_state
    }

    fn transition_to(&mut self, new_state: SpacecraftState, time: f64) {
        if self.current_state != new_state {
            println!(
                "State transition at t={:.2}s: {} -> {}",
                time, self.current_state, new_state
            );
            self.current_state = new_state;
            self.last_state_change = time;
        }
    }

    pub fn evaluate_transition<T: SpacecraftProperties>(&mut self, state: &VehicleState<T>) {
        let angular_velocity = state.angular_velocity.magnitude();
        let current_time = state.mission_elapsed_time;

        match self.current_state {
            SpacecraftState::SafeMode => {
                if angular_velocity > self.angular_velocity_threshold {
                    self.transition_to(SpacecraftState::Detumbling, current_time);
                }
            }
            SpacecraftState::Detumbling => {
                if angular_velocity < self.angular_velocity_threshold {
                    self.transition_to(SpacecraftState::NominalOperation, current_time);
                }
            }
            SpacecraftState::NominalOperation => {
                if angular_velocity > self.emergency_angular_velocity {
                    self.transition_to(SpacecraftState::Emergency, current_time);
                }
            }
            SpacecraftState::ManeuverPrep => self.evaluate_maneuver_prep(state, current_time),
            SpacecraftState::Maneuvering => self.evaluate_maneuvering(state, current_time),
            SpacecraftState::Emergency => self.evaluate_emergency(state, current_time),
        }
    }

    fn evaluate_maneuver_prep<T: SpacecraftProperties>(
        &mut self,
        vehicle_state: &VehicleState<T>,
        time: f64,
    ) {
        let angular_velocity = vehicle_state.angular_velocity.magnitude();

        if angular_velocity < self.angular_velocity_threshold && time - self.last_state_change > 5.0
        {
            // Minimum 5s prep time
            self.transition_to(SpacecraftState::Maneuvering, time);
        }
    }

    fn evaluate_maneuvering<T: SpacecraftProperties>(
        &mut self,
        vehicle_state: &VehicleState<T>,
        time: f64,
    ) {
        let angular_velocity = vehicle_state.angular_velocity.magnitude();

        if angular_velocity > self.emergency_angular_velocity {
            self.transition_to(SpacecraftState::Emergency, time);
        }
    }

    fn evaluate_emergency<T: SpacecraftProperties>(
        &mut self,
        vehicle_state: &VehicleState<T>,
        time: f64,
    ) {
        let angular_velocity = vehicle_state.angular_velocity.magnitude();

        if angular_velocity < self.angular_velocity_threshold
            && time - self.last_state_change > 30.0
        {
            // Minimum 30s in emergency
            self.transition_to(SpacecraftState::SafeMode, time);
        }
    }

    pub fn command_maneuver(&mut self, time: f64) -> bool {
        if self.current_state == SpacecraftState::NominalOperation {
            self.transition_to(SpacecraftState::ManeuverPrep, time);
            true
        } else if time - self.last_message_time > 10.0 {
            println!("Cannot start maneuver from state: {}", self.current_state);
            self.last_message_time = time;
            false
        } else {
            false
        }
    }

    pub fn should_apply_control(&self) -> bool {
        !matches!(
            self.current_state,
            SpacecraftState::SafeMode | SpacecraftState::Emergency
        )
    }

    pub fn should_apply_thrust(&self) -> bool {
        matches!(self.current_state, SpacecraftState::Maneuvering)
    }

    pub fn get_last_state_change(&self) -> f64 {
        self.last_state_change
    }
}
