#!/usr/bin/env python  
import roslib
import rospy
import tf
import math
import numpy as np
import tf2_ros


from d435_prueba.msg import GrupoObstaculos
from d435_prueba.msg import Obstaculo
from d435_prueba.msg import PuntosObstaculo
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import Temperature,LaserScan

laser_mod_pub = rospy.Publisher("scan_realsense", LaserScan, queue_size=200)
point_d435_pub = rospy.Publisher("point_d435", PointCloud, queue_size=200)

def cb_realsense(dato):   
    
    PointCloud_d435_msg = PointCloud()
    punto = Point32()
    PointCloud_d435_msg.header.frame_id="Realsenser_link"
    lim_inf = -3.14
    lim_sup = 3.14
    incremento = (lim_sup - lim_inf)/399
    
    laser_modi = LaserScan()
    laser_modi.header.frame_id="Realsenser_link"
    laser_modi.header.stamp = rospy.Time.now()
    laser_modi.angle_min = lim_inf
    laser_modi.angle_max = lim_sup
    laser_modi.angle_increment = incremento
    #laser_modi.time_increment = 0.1
    #laser_modi.scan_time  = laser.scan_time
    laser_modi.range_max = 100
    laser_modi.range_min = 0
    #laser_modi.intensities = laser.intensities
    
    angulo_lidar = lim_inf
    for opm in range(1,400):
        laser_modi.ranges.append(float('inf'))
    
    if len(dato.grupo_obstaculos)>0:
        for obstaculo in dato.grupo_obstaculos:
            for puntos_array in obstaculo.array_puntos:
                angulo_cloud = math.atan(-puntos_array.x/puntos_array.d)
                i = 0
                #print angulo_cloud
                while i < 400:
                    angulo_lidar_incrementado = angulo_lidar + incremento
                    #print "a lidar %f a cloud %f  a lidar_in %f" % (angulo_lidar,angulo_cloud,angulo_lidar_incrementado)   
                    #print  i
                    if (angulo_cloud > angulo_lidar) and (angulo_cloud < angulo_lidar_incrementado):
                        vector = np.array([puntos_array.d,-puntos_array.x,-puntos_array.y])
                        distancia_laser  = np.linalg.norm(vector)  
                        if math.isinf(laser_modi.ranges[i]):
                            laser_modi.ranges[i]=distancia_laser
                        else:
                            if laser_modi.ranges[i] > distancia_laser:
                                laser_modi.ranges[i]=distancia_laser 
                        #print "a lidar %f a cloud %f posi %f" % (angulo_lidar,angulo_cloud,i)
                    angulo_lidar = angulo_lidar + incremento
                    i = i+1
                angulo_lidar = lim_inf
                PointCloud_d435_msg.points.append(Point32(puntos_array.d,-puntos_array.x,-puntos_array.y))

        point_d435_pub.publish(PointCloud_d435_msg)
    laser_mod_pub.publish(laser_modi)
   

def publicador_de_odometrias():  
    rospy.init_node('point_d435_nodo')
    rospy.Subscriber("/pub_grupo_obs", GrupoObstaculos, cb_realsense,queue_size=100)
    rate = rospy.Rate(2)
    
    while not rospy.is_shutdown():
    
        rate.sleep()

if __name__ == '__main__':
    
    publicador_de_odometrias()
