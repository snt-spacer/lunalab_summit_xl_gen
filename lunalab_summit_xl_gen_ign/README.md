# lunalab_summit_xl_gen_ign

Ignition Gazebo-specific configuration of Summit XL-GEN (LunaLab variant).

## Instructions

### bridge

Communication between ROS 2 and Ignition Gazebo can be facilitated by utilising [ros_ign](https://github.com/ignitionrobotics/ros_ign/tree/ros2) bridge. In order to do so, a configurable [bridge.launch.py](./launch/bridge.launch.py) script is included to simplify the process for `lunalab_summit_xl_gen`. It can be launched separately or included in another launch script while passing the desired arguments.

```bash
ros2 launch lunalab_summit_xl_gen_ign bridge.launch.py <arg_i>:=<val_i>
```

To see all arguments, please use `ros2 launch --show-args lunalab_summit_xl_gen_ign bridge.launch.py`.

## Examples

### ex_follow_target

To see if everything is functioning properly, try using [ex_folow_target.launch.py](./launch/ex_folow_target.launch.py) script. It launches Ignition Gazebo, move_group of MoveIt 2 and ROS 2 \<-> IGN bridges that enable robot to follow a target. Simply start the simulation and move the target object around with Transform Control tool.

```bash
ros2 launch lunalab_summit_xl_gen_ign ex_folow_target.launch.py
```

In order to verify control of the move base, try running `teleop_twist_keyboard`.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/lunalab_summit_xl_gen/cmd_vel
```

## Directory Structure

The following directory structure is utilised for this package.

```bash
.
├── examples/ex_follow_target.py             # [dir] Examples (e.g. ex_follow_target.py)
├── launch/                                  # [dir] ROS 2 launch scripts
    ├── examples/                            # [dir] Launch scripts for examples (ex_follow_target.launch.py)
    └── bridge.launch.py                     # Configurable launch script for bridging communication between ROS 2 and Ignition
├── rviz/ign_moveit.rviz                     # Generic RViz2 config that includes tf2 visualisation and MoveIt 2 planning
├── worlds/moveit_follow_target.sdf          # SDF of world used for ex_follow_target example
├── CMakeLists.txt                           # Colcon-enabled CMake recipe
└── package.xml                              # ROS 2 package metadata
```
