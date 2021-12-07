# lunalab_summit_xl_gen_control

Configuration for control of Summit XL-GEN (LunaLab variant). Currently, only manipulator control is implemented within this module.

## Instructions

Launch script [j2s7s300_control.launch](launch/j2s7s300_control.launch) enables control of Kinova j2s7s300 by initialising action servers for Follow Joint Trajectory and Gripper Command. Hereafter, use an external application to send goals to these action servers.

```bash
roslaunch lunalab_summit_xl_gen_control j2s7s300_control.launch
```

## Directory Structure

The following directory structure is utilised for this package.

```bash
.
├── config/j2s7s300_parameters.yaml          # Driver-level configuration for j2s7s300
├── launch/j2s7s300_control.launch           # ROS launch script that enables control of j2s7s300 via actions
├── CMakeLists.txt                           # Catkin-enabled CMake recipe
└── package.xml                              # ROS package metadata
```
