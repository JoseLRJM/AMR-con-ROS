#!/usr/bin/env python  

import rospy
import yaml
import math
from nav_msgs.msg import OccupancyGrid
from rtabmap_ros.msg import Info
from sensor_msgs.msg import Temperature,LaserScan
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Float32,Int32MultiArray
import numpy as np
import os
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


landmarkid = 0.0
refid = 0.0
loopclosureid = 0.0
temperatura_camara = 0.0
corriente_consumida = 0.0
voltaje_bateria = 0.0
estado_paro_emergencia = 0.0
count_desubicacion_max = 3
vel_x = 0
vel_th = 0
odom_twist_x = 0
odom_twist_th = 0
count_desubicacion = 0

map_edited = OccupancyGrid()
info_robot_msg = Float32MultiArray()
set_estado_msg = Int32MultiArray()

map_modified_pub = rospy.Publisher("map_m", OccupancyGrid, latch=True,queue_size=200)
info_robot_pub = rospy.Publisher("info_robot", Float32MultiArray, queue_size=200)
laser_mod_pub = rospy.Publisher("scan_mod", LaserScan, queue_size=200)
set_estado_pub = rospy.Publisher("set_estado", Int32MultiArray, queue_size=1)


voltaje_fill = 48.0
corriente_fill = 0.0
bateria_consumida =0.0
porcentaje_bateria = 100.0

ema_alpha = 0.05
corriente_alpha = 0.4
capacidad_bateria = 60.0

andulo_maximo_filtro = 80
angulo_minimo_filtro = 100

def cb_twist_vel(data):
    global vel_x
    global vel_th
    vel_x = data.linear.x
    vel_th = data.angular.z

def cb_map(dato):
    global map_edited
    map_data=[]
    i=0
    maprecived = True

    for i_map in dato.data:
        if i_map == -1:
            map_data.append(100)
        else:
            map_data.append(i_map)
        i = i + 1

    tuple(map_data) 
    map_edited.header = dato.header
    map_edited.info = dato.info
    map_edited.data = tuple(map_data)
    map_modified_pub.publish(map_edited)
    
def cb_info_rtabmap(dato):
    global landmarkid
    global refid
    global loopclosureid
    global count_desubicacion

    refid=dato.refId
    loopclosureid=dato.loopClosureId
    landmarkid = dato.landmarkId
    print "refid %d loopclosure %d  proximitydete %d" % (refid,loopclosureid,dato.proximityDetectionId) 
    print  dato.wmState
    if len(dato.odom_cache.links)>0:
        for link in dato.odom_cache.links:
            landmarkid = link.type
            count_desubicacion = 0
            set_estado_msg.data = [3,0]
            set_estado_pub.publish(set_estado_msg)
        print landmarkid
    else:
        count_desubicacion = count_desubicacion + 1
        print "perdido"



def cb_temp_camara(dato):
    global temperatura_camara
    temperatura_camara=dato.temperature

def cb_dato_robot(dato):
    global porcentaje_bateria
    global corriente_consumida
    global voltaje_bateria
    global estado_paro_emergencia
    global voltaje_fill
    global corriente_fill
    global bateria_consumida
    global ema_alpha
    global corriente_alpha
    global capacidad_bateria

    voltaje_fill = ema_alpha * dato.data[1] + (1 - ema_alpha) * voltaje_fill
    corriente_fill = corriente_alpha * abs(2.3*dato.data[2]) + (1 - corriente_alpha) * corriente_fill
    bateria_consumida = bateria_consumida + abs (corriente_fill * 0.000055)
    porcentaje_bateria = 100*((capacidad_bateria-bateria_consumida)/capacidad_bateria)
    #porcentaje_bateria = dato.data[0]
    voltaje_bateria = dato.data[1] #dato.data[1]
    corriente_consumida = corriente_fill #dato.data[2]
    estado_paro_emergencia=dato.data[8]  

def cb_laser(laser):

    angulo = 0
    laser_modi = LaserScan()

    for punto in laser.ranges:
        
        angulo = angulo + math.degrees(laser.angle_increment)
        if angulo > 65 and angulo < 85:
            laser_modi.ranges.append(float('inf'))

        elif angulo > 240 and angulo < 265:
            laser_modi.ranges.append(float('inf'))
        
        else:
            laser_modi.ranges.append(punto)

    laser_modi.header = laser.header
    laser_modi.angle_min = laser.angle_min
    laser_modi.angle_max = laser.angle_max
    laser_modi.angle_increment = laser.angle_increment
    laser_modi.time_increment = laser.time_increment
    laser_modi.scan_time  = laser.scan_time
    laser_modi.range_max = laser.range_max
    laser_modi.range_min = laser.range_min
    laser_modi.intensities = laser.intensities
    laser_mod_pub.publish(laser_modi)

def cb_odom(odom_msg):
    global odom_twist_x
    global odom_twist_th
    odom_twist_x = odom_msg.twist.twist.linear.x
    odom_twist_th = odom_msg.twist.twist.angular.z


if __name__ == "__main__":
    
    rospy.init_node("map_edit")
    map_sub = rospy.Subscriber("/map", OccupancyGrid, cb_map,queue_size=100)
    info_rtabmap_sub = rospy.Subscriber("/info", Info, cb_info_rtabmap,queue_size=100)
    temp_camara_sub = rospy.Subscriber("/zed/temperature/left", Temperature, cb_temp_camara,queue_size=100)
    dato_robot_sub = rospy.Subscriber("/data_robot", Float32MultiArray, cb_dato_robot,queue_size=100)
    lase_sub = rospy.Subscriber("/scan", LaserScan, cb_laser,queue_size=100)
    cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, cb_twist_vel,queue_size=30)
    aodom_sub = rospy.Subscriber("/odometry/filtered", Odometry, cb_odom,queue_size=100)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        temp = os.popen("cat /sys/devices/virtual/thermal/thermal_zone0/temp").read()
        info_robot_msg.data = [vel_x,vel_th,odom_twist_x,odom_twist_th,landmarkid,refid,temperatura_camara,float(temp)/1000,estado_paro_emergencia]
        info_robot_pub.publish(info_robot_msg)
        #porcentaje_bateria,voltaje_bateria,corriente_consumida,loopclosureid
        if count_desubicacion > count_desubicacion_max:
            set_estado_msg.data = [3,1]
            set_estado_pub.publish(set_estado_msg)
            print "perdido"
        rate.sleep()