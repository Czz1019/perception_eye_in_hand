#!/usr/bin/env bash
# Source this in diagnostic terminals; the two launch scripts use it too.
GAZEBO_EIH_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-78}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11478}"
source /opt/ros/humble/setup.bash
source "$GAZEBO_EIH_WS/install/local_setup.bash"
echo "[EIH environment] ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"
