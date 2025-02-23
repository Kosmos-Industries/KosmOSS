# KosmOSS

[![Python CI](https://github.com/Kosmos-Industries/KosmOSS/actions/workflows/python.yml/badge.svg?branch=main)](https://github.com/Kosmos-Industries/KosmOSS/actions/workflows/python.yml)
[![Rust CI](https://github.com/Kosmos-Industries/KosmOSS/actions/workflows/rust.yml/badge.svg)](https://github.com/Kosmos-Industries/KosmOSS/actions/workflows/rust.yml)

Lightning fast aerospace software.

## Overview
KosmOSS is a comprehensive spacecraft simulation and control software stack written in Rust, with Python visualization tools. It includes:

- Low-fidelity orbital mechanics simulation with Earth gravity model
- Quaternion-based attitude dynamics and control
- Hohmann transfer guidance for orbit raising/lowering
- Real-time Earth orientation parameters (EOP) for coordinate transformations (pulled from Celestrak)
- Low-fidelity atmospheric drag model
- Interactive 3D mission visualization dashboard
- Energy and angular momentum conservation tracking

KosmOSS is currently in an alpha state, providing a 6DOF simulation environment for spacecraft dynamics and control.
Right now main.rs configures a simple ballistic trajectory for a spacecraft.

## Requirements

### Rust
- Rust 2021 edition or later
- Cargo package manager

### Python (for visualization)
- Python 3.10+
- Required packages:
  - matplotlib
  - numpy
  - pandas
  - cartopy
  - pytest

## Setup

1. Clone the repository:
```bash
git clone https://github.com/Kosmos-Industries/KosmOSS.git
cd KosmOSS
```

2. Install Python dependencies using Poetry:
```bash
poetry install
```

## Usage

### Running the simulation
```bash
cargo run --release
```

### Visualizing the results
```bash
poetry run python viz/plotMissionDashboard.py
```

This will generate both static and interactive visualization dashboards showing:
- 3D orbital trajectory
- Spacecraft attitude
- Control torques and thrust
- Energy and angular momentum conservation
- Ground track on Earth map

## Finite State Machine (FSM)

The spacecraft simulation includes a state machine that manages different operational modes:

### States
- **Safe Mode**: Initial state with minimal system activity
- **Detumbling**: Active attitude control to reduce angular velocity
- **Nominal Operation**: Normal orbital operations
- **Maneuver Preparation**: Pre-maneuver checks and preparation
- **Maneuvering**: Active orbital maneuver execution
- **Emergency**: Fault handling and recovery

### State Transitions
- Safe Mode → Detumbling: When angular velocity exceeds threshold
- Detumbling → Nominal Operation: When angular velocity is stabilized
- Nominal Operation → Maneuver Prep: When maneuver is commanded
- Maneuver Prep → Maneuvering: After preparation period
- Any State → Emergency: When angular velocity exceeds safety threshold
- Emergency → Safe Mode: After recovery period

### Control Authority
The FSM determines:
- When attitude control can be applied
- When orbital maneuvers can be executed
- Safety constraints for spacecraft operations

### Usage
The FSM automatically evaluates state transitions based on spacecraft dynamics and mission time. Maneuvers can only be commanded during Nominal Operation state.

Example state sequence:
Safe Mode → Detumbling → Nominal Operation → Maneuver Prep → Maneuvering → Nominal Operation

## Contributing

We welcome contributions! Please:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Run tests and formatting:
```bash
cargo test
cargo fmt
poetry run pytest
poetry run ruff format
```
5. Create a pull request

## License

This project is licensed under the Apache 2.0 License. See the LICENSE file for details.