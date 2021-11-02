#!/usr/bin/env bash
# This script converts xacro (URDF variant) into URDF for `lunalab_summit_xl_gen_description` package

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
XACRO_PATH="$(dirname ${SCRIPT_DIR})/urdf/lunalab_summit_xl_gen.urdf.xacro"
URDF_PATH="$(dirname ${SCRIPT_DIR})/urdf/lunalab_summit_xl_gen.urdf"

# Arguments for xacro
XACRO_ARGS="name:=lunalab_summit_xl_gen
            prefix:=robot_
            safety_limits:=true
            safety_soft_limit_margin:=0.17453293
            safety_k_position:=20
            high_quality_mesh:=true
           "

# Remove old URDF file
rm ${URDF_PATH}

# Process xacro into URDF
xacro ${XACRO_PATH} ${XACRO_ARGS} >${URDF_PATH} &&
    echo "Created new '${URDF_PATH}'"
