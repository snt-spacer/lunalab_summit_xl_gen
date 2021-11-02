# Summit XL-GEN (LunaLab)

Software packages for Summit XL-GEN that enable researchers to conduct experiments in a simulated environment and inside LunaLab.

This branch targets packages that are built around ROS 2. Furthermore, Ignition Gazebo is supported as the primary robotics simulator.

## Overview

Below is an overview of the included packages, with a small description of their purpose. For more information, please see README.md of each individual package.
- [**lunalab_summit_xl_gen_description**](./lunalab_summit_xl_gen_description) &ndash; URDF and SDF description of the robot
- [**lunalab_summit_xl_gen_moveit_config**](./lunalab_summit_xl_gen_moveit_config) &ndash; MoveIt 2 configuration for the robotic manipulator

## Instructions

### Requirements

- **OS:** Ubuntu 20.04 (Focal)
  - Other distributions might work, but they were not tested.

### Dependencies

These are the primary dependencies required to use this project.

- ROS 2 [Foxy](https://docs.ros.org/en/foxy/Installation.html) OR [Rolling (recommended)](https://docs.ros.org/en/rolling/Installation.html)
  - Support for [Galactic](https://docs.ros.org/en/galactic/Installation.html) will be added once it is released and binary packages are available. It will then become the recommended distribution to use.
- Ignition [Fortress](https://ignitionrobotics.org/docs/fortress)
- [ros_ign](https://github.com/ignitionrobotics/ros_ign/tree/ros2)
  - Install/build a version based on the selected combination of ROS 2 release and Ignition version
- [MoveIt 2](https://moveit.ros.org/install-moveit2/binary)
  - Install/build a version based on the selected ROS 2 release

Additional dependencies are pulled from git and built together with this repository, see [lunalab_summit_xl_gen.repos](lunalab_summit_xl_gen.repos) for more details.

### Building

Clone this repository and import VCS dependencies. Then install dependencies and build with [colcon](https://colcon.readthedocs.io).

```bash
# Create workspace for the project (can be skippid)
mkdir -p lunalab_summit_xl_gen_ws/src && cd lunalab_summit_xl_gen_ws/src
# Clone this repository
git clone ssh://git@gitlab.uni.lu:8022/spacer/phd/AndrejOrsula/lunalab_summit_xl_gen.git
# Import additional git dependencies
vcs import < lunalab_summit_xl_gen_ws/lunalab_summit_xl_gen.repos && cd ..
# Install external dependencies via rosdep
rosdep install -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
# Build with colcon
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

### Sourcing

Before utilising this package, remember to source the ROS 2 workspace overlay.

```bash
source lunalab_summit_xl_gen_ws/install/local_setup.bash
```

This enables:
- Execution of scripts and examples via `ros2 run lunalab_summit_xl_gen <executable>`
- Launching of setup scripts via `ros2 launch lunalab_summit_xl_gen <launch_script>`
- Discoverability of shared resources
