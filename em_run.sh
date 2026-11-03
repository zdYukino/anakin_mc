#!/usr/bin/env bash
set -euo pipefail
repo=$(cd -- "$(dirname -- "$(realpath -- "${BASH_SOURCE[0]}")")" && pwd)
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
 echo 'Usage: ./em_run.sh [ROS argument tokens, e.g. -p name:=value]'
 echo 'Starts MC only. Models default to the selected install; SDK defaults to ../anakin_sdk/install/anakin_sdk.'
 exit 0
fi
sdk_prefix=${ANAKIN_SDK_PREFIX:-"$repo/../anakin_sdk/install/anakin_sdk"}
prefix=${ANAKIN_PREFIX:-"$repo/install/anakin_mc"}
fail() { echo "anakin_mc: $*" >&2; exit 2; }
read_revision() {
 local path=$1
 local -a lines=()
 [[ -r "$path" ]] || fail "Missing SDK provenance: $path"
 mapfile -t lines < "$path"
 [[ ${#lines[@]} == 1 && ${lines[0]} =~ ^[0-9a-f]{40}$ ]] || fail "Invalid SDK provenance: $path"
 revision_value=${lines[0]}
}
prefix=$(realpath -e -- "$prefix") || fail "Missing selected MC prefix: $prefix"
sdk_prefix=$(realpath -e -- "$sdk_prefix") || fail "Missing selected SDK prefix: $sdk_prefix"
[[ -f "$prefix/share/ament_index/resource_index/packages/anakin_mc" ]] || fail 'Selected MC prefix has no local package marker'
[[ -f "$sdk_prefix/share/ament_index/resource_index/packages/anakin_sdk" ]] || fail 'Selected SDK prefix has no local package marker'
binary="$prefix/lib/anakin_mc/anakin_mc_node"
[[ -f "$binary" && -x "$binary" ]] || fail "Missing C++ binary: $binary"
[[ $(realpath -e -- "$binary") == "$binary" ]] || fail 'Selected ELF escapes its install prefix'
[[ $(od -An -tx1 -N4 -- "$binary" | tr -d ' \n') == 7f454c46 ]] || fail 'Selected executable is not an ELF'
read_revision "$repo/anakin_sdk.revision"
sdk_revision=$revision_value
read_revision "$prefix/share/anakin_mc/runtime-sdk.revision"
[[ "$revision_value" == "$sdk_revision" ]] || fail 'Binary was built against another SDK revision; rebuild this prefix'
metadata="$sdk_prefix/share/anakin_sdk/cmake/revision.cmake"
[[ -r "$metadata" ]] || fail "Missing installed SDK provenance: $metadata"
installed_revision=''
revision_pattern='^set\(ANAKIN_SDK_REVISION "([0-9a-f]{40})"\)$'
ignored_pattern='^[[:space:]]*(#|$)'
while IFS= read -r line || [[ -n "$line" ]]; do
 if [[ "$line" =~ $ignored_pattern ]]; then continue; fi
 [[ "$line" =~ $revision_pattern ]] || fail 'Invalid installed SDK revision declaration'
 [[ -z "$installed_revision" ]] || fail 'Duplicate installed SDK revision'
 installed_revision=${BASH_REMATCH[1]}
done < "$metadata"
[[ "$installed_revision" == "$sdk_revision" ]] || fail 'Installed SDK revision mismatch'
export ANAKIN_PREFIX="$prefix" ANAKIN_SDK_PREFIX="$sdk_prefix"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-87}
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros_prefix=${ROS_PREFIX:-/opt/ros/${ROS_DISTRO:-humble}}
export AMENT_PREFIX_PATH="$sdk_prefix:$prefix:$ros_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export LD_LIBRARY_PATH="$sdk_prefix/lib:$prefix/lib:$ros_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
ort=${ONNXRUNTIME_ROOT:-"$repo/.deps/onnxruntime-linux-x64-1.20.1"}
assets=${ANAKIN_ASSETS:-"$prefix/share/anakin_mc/assets"}
loco=${ANAKIN_LOCO_MODEL:-"$assets/models/g1/asap/dec_loco/20250109_231507-noDR_rand_history_loco_stand_height_noise-decoupled_locomotion-g1_29dof/model_6600.onnx"}
mimic=${ANAKIN_MIMIC_MODEL:-"$assets/models/g1/asap/mimic/CR7_level1/model_191500.onnx"}
[[ -f "$loco" && -f "$mimic" ]] || { echo 'Pinned model paths unavailable; set ANAKIN_LOCO_MODEL and ANAKIN_MIMIC_MODEL' >&2; exit 2; }
[[ -f "$ort/lib/libonnxruntime.so.1" && -r "$ort/VERSION_NUMBER" && $(< "$ort/VERSION_NUMBER") == 1.20.1 ]] || fail 'Missing ONNX Runtime 1.20.1; run scripts/setup_ort.py or set ONNXRUNTIME_ROOT'
export LD_LIBRARY_PATH="$ort/lib:$LD_LIBRARY_PATH"
export ROS_LOG_DIR=${ROS_LOG_DIR:-"$repo/log/ros"}
if [[ ${ANAKIN_LAUNCH_CHECK_ONLY:-0} == 1 ]]; then exit 0; fi
mkdir -p -- "$ROS_LOG_DIR"
exec "$binary" --ros-args -p "loco_model:=$loco" -p "mimic_model:=$mimic" "$@"
