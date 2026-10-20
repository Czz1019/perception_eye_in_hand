#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
exec ros2 run tf2_ros static_transform_publisher --frame-id tool0 --child-frame-id camera_link --x -0.0406379061113624 --y -0.05902311421350448 --z -0.012384254831755863 --qx 0.0025872583947125077 --qy -0.002933202909281628 --qz 0.9251005977793049 --qw -0.3797019705019513
