# lunalab_summit_xl_gen

Metapackage for Summit XL-GEN (LunaLab variant).

## Functionality

During the build stage, this package converts xacros of [lunalab_summit_xl_gen_description](../lunalab_summit_xl_gen_description) and [lunalab_summit_xl_gen_moveit_config](../lunalab_summit_xl_gen_moveit_config) into auto-generated URDF, SDF and SRDF descriptions for convenience.

## Directory Structure

The following directory structure is utilised for this package.

```bash
.
├── CMakeLists.txt # Colcon-enabled CMake recipe
└── package.xml    # ROS 2 package metadata
```
