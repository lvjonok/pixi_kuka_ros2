#!/usr/bin/env bash

export ROS_DISTRO="jazzy"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_HOME="${ROS_HOME:-${PWD}/.ros}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${PWD}/log/ros}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CRISP_CONFIG_PATH="${CRISP_CONFIG_PATH:-${PWD}/config}"

if [[ -f "install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "install/setup.bash"
fi
