#!/usr/bin/env bash
# This script converts xacro (URDF variant) into URDF for `lunalab_summit_xl_gen_description` package

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
XACRO_PATH="$(dirname ${SCRIPT_DIR})/urdf/lunalab_summit_xl_gen.urdf.xacro"
URDF_PATH="$(dirname ${SCRIPT_DIR})/urdf/lunalab_summit_xl_gen.urdf"

# Arguments for xacro
XACRO_ARGS="
name:=lunalab_summit_xl_gen
prefix:=robot_
safety_limits:=true
safety_soft_limit_margin:=0.17453293
safety_k_position:=20
collision_chassis:=true
collision_wheels:=true
collision_arm:=true
collision_gripper:=true
high_quality_mesh:=true
mimic_gripper_joints:=false
gazebo_preserve_fixed_joint:=false
gazebo_self_collide_fingers:=true
gazebo_ros_planar_move_plugin:=false
gazebo_ros_control_plugin:=false
gazebo_ros_ft_sensor:=false
"

# Remove old URDF file
rm ${URDF_PATH} 2>/dev/null

# Process xacro into URDF
xacro ${XACRO_PATH} ${XACRO_ARGS} -o ${URDF_PATH} &&
    echo "Created new '${URDF_PATH}'"

# Add to stating area
git add ${URDF_PATH}
