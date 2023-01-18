#!/usr/bin/env python  
import roslib
import rospy
from std_msgs.msg import Int32,UInt32MultiArray,Int32MultiArray,String,Bool,Float32MultiArray,Int16MultiArray
from os import path
import os.path
import time


hay_mision_cargada=False
slam_lanzado = False
modo_auto_activo = False
altura_stacker = 0.0


respuesta_robot = rospy.Publisher("respuesta_robot", Int32MultiArray, queue_size=200)


def cb_publicador_app(dato):
    global slam_lanzado
    global modo_auto_activo
    respuesta_robot_msg = Int32MultiArray()

    if dato.data == 6: #iniciar mision modo auto
        if not modo_auto_activo:
            modo_auto_activo = True
            print "modo auto activando"
            time.sleep(0.3)
            respuesta_robot_msg.data = [9,1]
            while respuesta_robot.get_num_connections() < 1:
                print "esperando a conectarse"
            respuesta_robot.publish(respuesta_robot_msg)
            os.system("gnome-terminal --tab -- roslaunch champy_2-0 navtest.launch")
        else:
            print "modo auto YA esta activo"
            time.sleep(0.3)
            respuesta_robot_msg.data = [9,1]
            while respuesta_robot.get_num_connections() < 1:
                print "esperando a conectarse"
            respuesta_robot.publish(respuesta_robot_msg)

    elif dato.data == 7: #matar modo auto 
        modo_auto_activo = False
 
if __name__ == "__main__":
    rospy.init_node("lanzar_nodos")
    rospy.Subscriber("/publicador_app", Int32, cb_publicador_app,queue_size=100)
    rospy.spin() 
