#!/usr/bin/env bash
# This script converts xacro (URDF variant) into SDF for `lunalab_summit_xl_gen_description` package

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
XACRO_PATH="$(dirname ${SCRIPT_DIR})/urdf/lunalab_summit_xl_gen.urdf.xacro"
SDF_PATH="$(dirname ${SCRIPT_DIR})/lunalab_summit_xl_gen/model.sdf"
TMP_URDF_PATH="/tmp/lunalab_summit_xl_gen_tmp.urdf"

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

# Remove old SDF file
rm ${SDF_PATH} 2>/dev/null

# Process xacro into URDF, then convert URDF to SDF and edit the SDF to use relative paths for meshes
xacro ${XACRO_PATH} ${XACRO_ARGS} -o ${TMP_URDF_PATH} &&
    gz sdf -p ${TMP_URDF_PATH} | sed 's/model:\/\/lunalab_summit_xl_gen_description\///g' >${SDF_PATH} &&
    echo "Created new '${SDF_PATH}'"

# Remove temporary URDF file
rm ${TMP_URDF_PATH} 2>/dev/null

# Add to stating area
git add ${SDF_PATH}
