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

    if dato.data == 4: #lanzar slam mapeo
        if not slam_lanzado:
            slam_lanzado = True
            print "lanzando slam mapeo"
            time.sleep(0.3)
            respuesta_robot_msg.data = [0,1]
            while respuesta_robot.get_num_connections() < 1:
                print "esperando a conectarse"
            respuesta_robot.publish(respuesta_robot_msg)
            respuesta_robot.publish(respuesta_robot_msg)
            os.system("gnome-terminal --tab -- roslaunch champy_2-0 slam_mapeo.launch")
            
        else:
            time.sleep(0.3)
            print "slam mapeo YA esta activo"
            respuesta_robot_msg.data = [0,1]
            while respuesta_robot.get_num_connections() < 1:
                print "esperando a conectarse"
            respuesta_robot.publish(respuesta_robot_msg)
            respuesta_robot.publish(respuesta_robot_msg)

    elif dato.data == 8: #lanzar slam localizacion
        if not slam_lanzado:
            slam_lanzado=True
            print "lanzando slam localizacion"
            time.sleep(0.3)
            respuesta_robot_msg.data = [8,1]
            while respuesta_robot.get_num_connections() < 1:
                print "esperando a conectarse"
            respuesta_robot.publish(respuesta_robot_msg)
            respuesta_robot.publish(respuesta_robot_msg)
            os.system("gnome-terminal --tab -- roslaunch champy_2-0 slam_localizacion.launch")

        else:
            print "slam localizacion YA esta activo"
            time.sleep(0.3)
            respuesta_robot_msg.data = [8,1]
            while respuesta_robot.get_num_connections() < 1:
                print "esperando a conectarse"
            respuesta_robot.publish(respuesta_robot_msg)
            respuesta_robot.publish(respuesta_robot_msg)

    elif dato.data == 5: #matar slam 
        slam_lanzado=False
 
if __name__ == "__main__":
    rospy.init_node("lanzar_nodos")
    rospy.Subscriber("/publicador_app", Int32, cb_publicador_app,queue_size=100)
    
    rospy.spin() 
