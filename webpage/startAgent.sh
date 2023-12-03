#!/bin/bash
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6 