#!/bin/bash
source ~/catkin_ws/devel/setup.bash
source /etc/ros/env.sh
export ROS_HOME=$(echo ~crbt/.ros

roslaunch champy_2-0 champy_hardware.launch
