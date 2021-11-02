#!/usr/bin/env bash
# This script converts xacro (URDF variant) into SDF for `lunalab_summit_xl_gen_description` package

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
XACRO_PATH="$(dirname ${SCRIPT_DIR})/urdf/lunalab_summit_xl_gen.urdf.xacro"
SDF_PATH="$(dirname ${SCRIPT_DIR})/lunalab_summit_xl_gen/model.sdf"
TMP_URDF_PATH="/tmp/lunalab_summit_xl_gen_tmp.urdf"

# Arguments for xacro
XACRO_ARGS="name:=lunalab_summit_xl_gen
            prefix:=robot_
            safety_limits:=true
            safety_soft_limit_margin:=0.17453293
            safety_k_position:=20
            high_quality_mesh:=true
           "

# Remove old SDF file
rm ${SDF_PATH}

# Process xacro into URDF, then convert URDF to SDF
xacro ${XACRO_PATH} ${XACRO_ARGS} >${TMP_URDF_PATH} &&
    ign sdf -p ${TMP_URDF_PATH} >${SDF_PATH} &&
    echo "Created new '${SDF_PATH}'"

# Edit SDF to use relative paths for meshes
sed -i -e 's/model:\/\/lunalab_summit_xl_gen_description\///g' ${SDF_PATH}

# Remove temporary URDF file
rm ${TMP_URDF_PATH}
