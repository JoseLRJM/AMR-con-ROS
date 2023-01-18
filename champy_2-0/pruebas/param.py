#!/usr/bin/env python  

from std_msgs.msg import Float32MultiArray
import roslib
import rospy
import time

vel_deseada_pub = rospy.Publisher("/set_mission", Float32MultiArray, queue_size=100)

if __name__ == "__main__":
    rospy.init_node('automatic_parking_vision')
    loop_rate = rospy.Rate(0.1)
    pu=Float32MultiArray()
    i=0
    while not rospy.is_shutdown():
        pu.data=[2,i,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18]
        vel_deseada_pub.publish(pu)
        pu.data=[1,i,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18]
        vel_deseada_pub.publish(pu)
        pu.data=[0,i,i]
        vel_deseada_pub.publish(pu)
        i=i+1
        loop_rate.sleep()