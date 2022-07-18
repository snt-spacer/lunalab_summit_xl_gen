# Summit XL-GEN (LunaLab)

Software packages for Summit XL-GEN with Kinova Gen2 arm. This repository enables researchers to conduct experiments in simulations and on a real robot inside the LunaLab facility at the University of Luxembourg.

<p align="left" float="middle">
  <img width="50.0%" src="https://user-images.githubusercontent.com/22929099/176204798-80665afb-50ba-463d-bcf4-2c2f06441e09.png" alt="Visualisation of lunalab_summit_xl_gen in a simulated lunar environment"/>
</p>

## Overview

This branch targets ROS 2 `galactic` and Gazebo `fortress`.

Below is an overview of the included packages, with a short description of their purpose. For more information, please see README.md of each individual package.

- [**lunalab_summit_xl_gen**](./lunalab_summit_xl_gen) – Metapackage
- [**lunalab_summit_xl_gen_description**](./lunalab_summit_xl_gen_description) – URDF and SDF description of the robot
- [**lunalab_summit_xl_gen_ign**](./lunalab_summit_xl_gen_ign) – Additional Gazebo-specific configuration of the robot
- [**lunalab_summit_xl_gen_moveit_config**](./lunalab_summit_xl_gen_moveit_config) – MoveIt 2 configuration for the robotic manipulator

## Instructions

### Dependencies

These are the primary dependencies required to use this project.

- ROS 2 [Galactic](https://docs.ros.org/en/galactic/Installation.html)
- Gazebo [Fortress](https://gazebosim.org/docs/fortress)

All additional dependencies are either pulled via [vcstool](https://wiki.ros.org/vcstool) ([lunalab_summit_xl_gen.repos](./lunalab_summit_xl_gen.repos)) or installed via [rosdep](https://wiki.ros.org/rosdep) during the building process below.

### Building

Clone this repository, import dependencies, install dependencies and build with [colcon](https://colcon.readthedocs.io).

```bash
# Clone this repository into your favourite ROS 2 workspace
git clone https://github.com/snt-spacer/lunalab_summit_xl_gen.git
# Import dependencies
vcs import < lunalab_summit_xl_gen/lunalab_summit_xl_gen.repos
# Install dependencies
IGNITION_VERSION=fortress rosdep install -y -r -i --rosdistro ${ROS_DISTRO} --from-paths .
# Build
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

### Sourcing

Before utilising this package, remember to source the ROS 2 workspace.

```bash
source install/local_setup.bash
```

This enables:

- Execution of binaries, scripts and examples via `ros2 run lunalab_summit_xl_gen_* <executable>`
- Launching of setup scripts via `ros2 launch lunalab_summit_xl_gen_* <launch_script>`
- Discoverability of shared resources
