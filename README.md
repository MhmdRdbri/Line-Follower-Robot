# Line-Follower-Robot

A Python-based line follower robot simulation using Webots, capable of navigating obstacles and scoring goals. 

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
  - [Usage](#usage)
- [Project Structure](#project-structure)
- [Contributing](#contributing)
- [License](#license)

# Overview

This project simulates a line follower robot using the Webots simulation environment and Python. The robot is designed to follow a designated path, navigate around obstacles, and achieve specific goals within the simulation. Line follower robots are widely used in robotics education and competitions to demonstrate autonomous navigation capabilities.

# Features

* Autonomous Navigation: Follows predefined paths using line detection algorithms.
* Obstacle Avoidance: Detects and navigates around obstacles seamlessly.
* Goal-Oriented Behavior: Designed to reach specific targets within the simulation environment.
* Modular Codebase: Structured for easy modifications and feature expansions.

# Getting Started

Follow these instructions to set up and run the simulation on your local machine.

# Prerequisites

* Webots R2025a: Ensure you have the latest version of Webots installed.

* Python 3.8+: The simulation scripts are written in Python.


## Installation

1. Clone the Repository:

```bash
git clone https://github.com/MhmdRdbri/Line-Follower-Robot.git
cd Line-Follower-Robot
```

2. Install Python Dependencies:

```bash
python -m venv env
source env/bin/activate  # On Windows: env\Scripts\activate
```
Then, install the required packages:

```bash
pip install -r requirements.txt
```


## Usage

1. Open the Simulation in Webots.
2. Run the Simulation
3. Monitor Robot Behavior


## Project Structure
```
Line-Follower-Robot/
├── controllers/
│   └── line_follower/
│       ├── line_follower.py
│       └── ...
├── worlds/
│   ├── line_follower_world.wbt
│   └── ...
├── README.md
└── requirements.txt
```



## Contributing

Contributions are welcome! Please open an issue or submit a pull request for any improvements or bug fixes.

## License

This project is licensed under the MIT License. See the LICENSE file for details.
