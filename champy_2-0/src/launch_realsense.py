#!/usr/bin/env python  
import roslib
import rospy
from os import path
import os.path
import time



if __name__ == "__main__":
    rospy.init_node("launch_realsense_node")
    os.system("cd /home/user/catkin_ws/src/d435_prueba/scripts; rosrun d435_prueba detector_pruebas_planta_live.py")
    rospy.spin() 
