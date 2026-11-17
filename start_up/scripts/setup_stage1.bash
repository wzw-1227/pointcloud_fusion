#!/usr/bin/env bash
# Source in each terminal; sourcing catkin setup alone drops PX4's source paths.
stage1_setup_environment() {
  local stage1_workspace="${STAGE1_WORKSPACE:-/home/yang/wzw/multi_robot_ws}"
  local stage1_lio_workspace="${STAGE1_LIO_WORKSPACE:-/home/yang/wzw/ct_ws}"
  local stage1_px4_dir="${STAGE1_PX4_DIR:-/home/yang/wzw/PX4_Firmware}"
  if [[ ! -f "$stage1_workspace/devel/setup.bash" ]]; then
    echo "Build the Stage 1 workspace first: $stage1_workspace" >&2
    return 1
  fi
  source "$stage1_workspace/devel/setup.bash" || return 1
  export ROS_PACKAGE_PATH="$stage1_px4_dir:$stage1_px4_dir/Tools/sitl_gazebo:${ROS_PACKAGE_PATH:-}"
  export GAZEBO_PLUGIN_PATH="$stage1_workspace/devel/lib:$stage1_lio_workspace/devel/lib:$stage1_px4_dir/build/px4_sitl_default/build_gazebo:${GAZEBO_PLUGIN_PATH:-}"
  export GAZEBO_MODEL_PATH="$stage1_workspace/src/start_up/models:$stage1_px4_dir/Tools/sitl_gazebo/models:${GAZEBO_MODEL_PATH:-}"
  export LD_LIBRARY_PATH="$stage1_px4_dir/build/px4_sitl_default/build_gazebo:${LD_LIBRARY_PATH:-}"
}
stage1_setup_environment
unset -f stage1_setup_environment
