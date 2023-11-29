#!/bin/zsh
source ~/microros_ws/install/setup.zsh
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6 &
ros2 launch rosbridge_server rosbridge_websocket_launch.xml 