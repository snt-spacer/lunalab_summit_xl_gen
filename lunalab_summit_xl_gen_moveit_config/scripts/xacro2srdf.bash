#!/usr/bin/env bash
# This script converts xacro (SRDF variant) into SRDF for `lunalab_summit_xl_gen_description` package

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
XACRO_PATH="$(dirname "${SCRIPT_DIR}")/srdf/lunalab_summit_xl_gen.srdf.xacro"
SRDF_PATH="$(dirname "${SCRIPT_DIR}")/srdf/lunalab_summit_xl_gen.srdf"

# Arguments for xacro
XACRO_ARGS=(
    name:=lunalab_summit_xl_gen
    prefix:=robot_
    external_devices:=false
    realsense_d435:=false
    realsense_d455:=false
    realsense_l515:=false
    lidar:=false
)

# Remove old SRDF file
rm "${SRDF_PATH}" 2>/dev/null

# Process xacro into SRDF
xacro "${XACRO_PATH}" "${XACRO_ARGS[@]}" -o "${SRDF_PATH}" &&
echo "Created new ${SRDF_PATH}"
