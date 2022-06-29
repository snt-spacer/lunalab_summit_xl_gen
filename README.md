# Summit XL-GEN (LunaLab)

Software packages for Summit XL-GEN that enable researchers to conduct experiments in a simulated environment and inside LunaLab.

This branch targets packages that are built around ROS 2. Furthermore, Ignition Gazebo is supported as the primary robotics simulator.

## Overview

Below is an overview of the included packages, with a small description of their purpose. For more information, please see README.md of each individual package.

Below is an overview of the included packages, with a short description of their purpose. For more information, please see README.md of each individual package.

- [**lunalab_summit_xl_gen**](./lunalab_summit_xl_gen) – Metapackage
- [**lunalab_summit_xl_gen_description**](./lunalab_summit_xl_gen_description) – URDF and SDF description of the robot
- [**lunalab_summit_xl_gen_ign**](./lunalab_summit_xl_gen_ign) – Additional Ignition Gazebo-specific configuration of the robot
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

Before utilising this package, remember to source the ROS 2 workspace overlay.

```bash
source lunalab_summit_xl_gen_ws/install/local_setup.bash
```

This enables:

- Execution of scripts and examples via `ros2 run lunalab_summit_xl_gen_* <executable>`
- Launching of setup scripts via `ros2 launch lunalab_summit_xl_gen_* <launch_script>`
- Discoverability of shared resources

## Connecting to Real Robot

Below are some general instructions oh how to connect to the real Summit XL-GEN.

### On-board router (local network of Summit XL-GEN)

- SSID: `SXL00-200505AA`
- Password:

### Internal compute unit

- IPv4: `192.168.0.200`
- hostname: `SXL00-200505AA`
- user: `summit`
- Password:

### NVIDIA Jetson Xavier NX Developer Kit

- IPv4: `192.168.0.150`
- hostname: `NJX18`
- user: `spacer`
- Password:

### Connecting over SSH

Once connected Summit XL-GEN network, you connect to the internal compute unit of Summit over SSH.

```bash
# Internal compute unit
ssh summit@192.168.0.200
# Xavier
ssh spacer@192.168.0.150
```

#### (Optional) Configuration of host and SSH alias

If desired, you can register these devices as hosts and configure their aliases for SSH.

```bash
# Add alias for `summit` host
echo -e "\n# Summit XL-GEN (LunaLab)\n192.168.0.200 SXL00-200505AA summit" | sudo tee -a /etc/hosts
# Add alias for `xavier` host
echo -e "\n# NVIDIA Jetson Xavier NX Developer Kit (LunaLab, mounted on Summit XL-GEN)\n192.168.0.150 NJX18 xavier" | sudo tee -a /etc/hosts
```

```bash
# Add `lunalab_summit` to SSH config
echo -e "\n# Summit XL-GEN (LunaLab)\nHost lunalab_summit\n\tHostname summit\n\tUser summit" | tee -a ${HOME}/.ssh/config
# Add `lunalab_xavier` to SSH config
echo -e "\n# NVIDIA Jetson Xavier NX Developer Kit (LunaLab, mounted on Summit XL-GEN)\nHost lunalab_xavier\n\tHostname xavier\n\tUser spacer" | tee -a ${HOME}/.ssh/config
```

Once enabled, you can SSH into the system using the `lunalab_summit` or `lunalab_xavier` alias.

```bash
# Internal compute unit
ssh lunalab_summit
# Xavier
ssh lunalab_xavier
```

### Firewall

If you experience some communication issues even though all nodes and their topics/services/actions are visible, you might need to disable firewall. Instead of making it completely inactive, allow communication for specific IP adresses.

Below is an example of how to achieve this for Uncomplicated Firewall (UFW).

```bash
# Restrict these rules to the specific network interface (e.g. wlp0s20f3)
export NETWORK_INTERFACE=wlp*
# Allow all communication with IP address of internal compute unit
sudo ufw allow in on ${NETWORK_INTERFACE} from 192.168.0.200
sudo ufw allow in on ${NETWORK_INTERFACE} to 192.168.0.200
# Allow all communication with IP address of Xavier
sudo ufw allow in on ${NETWORK_INTERFACE} from 192.168.0.150
sudo ufw allow in on ${NETWORK_INTERFACE} to 192.168.0.150
```
