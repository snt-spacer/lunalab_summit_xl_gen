# lunalab_summit_xl_gen_moveit_config

MoveIt configuration for manipulators that can be attached to Summit XL-GEN (LunaLab variant).

## Instructions

### SRDF

For SRDF, [lunalab_summit_xl_gen.srdf.xacro](./srdf/lunalab_summit_xl_gen.srdf.xacro) is the primary descriptor that includes all other xacros and creates a configuration based on the passed arguments. To generate SRDF out of xacro, you can use the included [xacro2srdf.bash](./scripts/xacro2srdf.bash) script and modify its arguments as needed. Once executed, [lunalab_summit_xl_gen.srdf](./srdf/lunalab_summit_xl_gen.srdf) will automatically be replaced. Alternatively, `xacro lunalab_summit_xl_gen.srdf.xacro name:="lunalab_summit_xl_gen" <arg_i>:=<val_i> ...` can be executed directly, e.g. this is preferred within any launch script.

## Directory Structure

The following directory structure is utilised for this package.

```bash
.
├── config/                                  # [dir] Configuration files for MoveIt
    ├── controllers_*.yaml                   # Configuration of ROS controllers for different command interfaces
    ├── joint_limits.yaml                    # List of velocity and acceleration joint limits
    ├── kinematics.yaml                      # Configuration for the kinematic solver
    ├── moveit_controller_manager.yaml       # List of controllers with their type and action namespace for use with MoveIt
    ├── ompl_planning.yaml                   # Configuration of OMPL planning and specific planners
    └── servo.yaml                           # Configuration for moveit_servo
├── launch/                                  # [dir] ROS launch scripts
├── rviz/moveit.rviz                         # RViz config for motion planning with MoveIt
├── scripts/                                 # [dir] Additional useful scripts
├── srdf/                                    # [dir] SRDF description (xacros)
    ├── lunalab_summit_xl_gen_j2s7s300.xacro # Xacro specific to SRDF of Kinova j2s7s300 manipulator
    ├── lunalab_summit_xl_gen.srdf           # SRDF generated from `lunalab_summit_xl_gen.srdf.xacro`
    ├── lunalab_summit_xl_gen.srdf.xacro     # The primary xacro of the robot
    └── lunalab_summit_xl_manipulators.xacro # Xacro for utilised manipulators
├── CMakeLists.txt                           # Catkin-enabled CMake recipe
└── package.xml                              # ROS package metadata
```
