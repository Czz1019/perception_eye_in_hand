#!/usr/bin/env bash
set -eo pipefail
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
unset AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX
source /opt/ros/humble/setup.bash
source /home/wqj/ur_arm/ros_ur_driver/install/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONDONTWRITEBYTECODE=1
exec ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur7e robot_ip:=192.168.56.100 reverse_ip:=192.168.56.38 \
  kinematics_params_file:=/home/wqj/ur_arm/my_robot_calibration.yaml \
  use_fake_hardware:=false headless_mode:=false \
  activate_joint_controller:=false launch_rviz:=false
