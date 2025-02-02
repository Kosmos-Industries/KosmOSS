# KosmOSS
Lightning fast aerospace software.

## Overview
KosmOSS is a comprehensive spacecraft simulation and control software stack written in Rust, with Python visualization tools. It includes:

- Orbital mechanics simulation
- Attitude dynamics and control
- Hohmann transfer guidance
- Mission visualization dashboard

KosmOSS is currenlty in a sub alpha state and is only a low fidelity 6DOF simulation for spacecraft for the time being.
This only showcases a few hours of development time and will be improved and iterated on.
Any suggestions are welcome! Email me at nnatsoulas@gmail.com

## Requirements

### Rust
- Rust 2021 edition or later
- Cargo package manager

### Python (for visualization)
- Python 3.7+
- Required packages:
  - matplotlib
  - numpy
  - pandas
  - cartopy
  - mpl_toolkits

## Setup
This thing works right out of the box :).

1. Ensure Rust and Cargo are installed.

2. Install Python dependencies:
```bash
pip install matplotlib numpy pandas cartopy mpl_toolkits
```

## Usage

### Running the simulation
```bash
cargo run --release
```

### Visualizing the results
```bash
python viz/plotMissionDashboard.py
```

## Contributing

We welcome contributions! Please fork the repository and create a pull request with your changes.

## License

This project is licensed under the Apache 2.0 License. See the LICENSE file for details.