#!/usr/bin/env bash
set -eo pipefail
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 PYTHONDONTWRITEBYTECODE=1
exec ros2 launch realsense2_camera rs_launch.py \
 camera_namespace:=camera camera_name:=camera serial_no:=_349622072343 \
 enable_depth:=true enable_color:=true rgb_camera.color_profile:=640x480x15 \
 enable_infra1:=false enable_infra2:=false enable_gyro:=false enable_accel:=false \
 pointcloud.enable:=false align_depth.enable:=false depth_module.depth_profile:=640x480x15
