import rospy
import numpy as np
import tf
from std_msgs.msg import UInt32MultiArray, Int32
from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math


def cb_GetMarkerOdom(markers_odom_msg):
    for marker_odom_msg in markers_odom_msg.markers:
        if marker_odom_msg.id == MARKER_ID_DETECTION:
            if is_marker_pose_received == False:
                    is_marker_pose_received = True
                diff_tag_time = rospy.get_time() - lasttime_tag
                lasttime_tag = rospy.get_time()
                quaternion = (marker_odom_msg.pose.pose.orientation.x, marker_odom_msg.pose.pose.orientation.y, marker_odom_msg.pose.pose.orientation.z, marker_odom_msg.pose.pose.orientation.w)
                theta = tf.transformations.euler_from_quaternion(quaternion)[2]

                mark_pos_theta = theta + np.pi / 2.
                mark_pos_x = marker_odom_msg.pose.pose.position.x
                mark_pos_y = marker_odom_msg.pose.pose.position.y

                #print('mar_th %f  mar_x %f  mar_y %f' % (self.mark_pos_theta,self.mark_pos_x, self.mark_pos_y))

            if inicializado: 

                    ofset_odom_x = self.odom_pos_x
                    self.ofset_odom_y = self.odom_pos_y
                    self.ofset_odom_theta = -(self.mark_pos_theta)-self.odom_theta

                    self.x_pro = (self.mark_pos_x * math.cos(self.mark_pos_theta)) + (self.mark_pos_y * math.sin(self.mark_pos_theta))
                    self.y_pro = (self.mark_pos_y * math.cos(self.mark_pos_theta)) - (self.mark_pos_x * math.sin(self.mark_pos_theta))

def cb_GetRobotOdom(self, robot_odom_msg):
        if self.is_odom_received == False:
            self.is_odom_received = True 
        quaternion = (robot_odom_msg.pose.pose.orientation.x, robot_odom_msg.pose.pose.orientation.y, robot_odom_msg.pose.pose.orientation.z, robot_odom_msg.pose.pose.orientation.w)
        
        self.odom_theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        self.odom_pos_x = robot_odom_msg.pose.pose.position.x
        self.odom_pos_y = robot_odom_msg.pose.pose.position.y
        
        if self.inicializado:

            self.th = self.odom_theta + self.ofset_odom_theta
            self.x = ((self.odom_pos_x - self.ofset_odom_x) * math.cos(self.ofset_odom_theta) - (self.odom_pos_y - self.ofset_odom_y) * math.sin(self.ofset_odom_theta)) - self.x_pro 
            self.y = ((self.odom_pos_y - self.ofset_odom_y) * math.cos(self.ofset_odom_theta) + (self.odom_pos_x - self.ofset_odom_x) * math.sin(self.ofset_odom_theta)) - self.y_pro
            
            if self.th > np.pi:
                self.th=-(2*np.pi - self.th)
            if self.th < -np.pi:
                self.th= 2*np.pi + self.th
            #print('th %f  x %f  y %f ' % (self.th,self.x, self.y))


def parking():
    rospy.init_node('parking_test')
    sub_odom_robot = rospy.Subscriber('/zed/odom', Odometry, self.cb_GetRobotOdom, queue_size = 1)
    sub_info_marker = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.cb_GetMarkerOdom, queue_size = 1)
    pub_cmd_vel = rospy.Publisher('/planner/cmd_vel', Twist, queue_size=1)