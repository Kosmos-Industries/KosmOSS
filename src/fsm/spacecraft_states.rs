use std::fmt;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SpacecraftState {
    SafeMode,
    Detumbling,
    NominalOperation,
    ManeuverPrep,
    Maneuvering,
    Emergency,
}

impl fmt::Display for SpacecraftState {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            SpacecraftState::SafeMode => write!(f, "Safe Mode"),
            SpacecraftState::Detumbling => write!(f, "Detumbling"),
            SpacecraftState::NominalOperation => write!(f, "Nominal Operation"),
            SpacecraftState::ManeuverPrep => write!(f, "Maneuver Preparation"),
            SpacecraftState::Maneuvering => write!(f, "Maneuvering"),
            SpacecraftState::Emergency => write!(f, "Emergency"),
        }
    }
} 