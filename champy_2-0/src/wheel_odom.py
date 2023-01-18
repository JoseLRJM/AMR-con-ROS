#!/usr/bin/env python  
import roslib
import rospy
import tf
import math
import tf2_ros
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from tf.transformations import *
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Float32,Int32
from sensor_msgs.msg import Imu
import numpy
from d435_prueba.msg import GrupoObstaculos
from d435_prueba.msg import Obstaculo
from d435_prueba.msg import PuntosObstaculo
#from geometry_msgs.msg import PoseWithCovarianceStamped
#from geometry_msgs.msg import TwistWithCovariance

#parametros fisicos
d_wheels= 0.53 #0.56
k_vel_x = 0.97
rad_wheels=0.255/2

#inicializacion de variables
th_odom=0.0
x_odom=0.0
y_odom=0.0
right_wheel_pose=0.0
left__wheel_pose=0.0
dt_ant=0.0 
vel_deseada = 3
vel_deseada_th = 6
vel_anterior_pausa=0
modo_control = 0
modo_control_ant = 0
senalizacion_robot = 0
#objetos de estructuras 
#twist_zed = Twist()
#odom_zed_map=Odometry()
odom = Odometry()
vel_wheel_msg = Float32MultiArray()
vel_msg = Float32MultiArray()
luces_msg = Int32MultiArray()

verde_pin = 4 #9
rojo_pin = 6 #11
amarillo_pin = 22 #12
buzz_pin = 13 #13

vel_left=0
vel_right=0    
espacio_libre=True
paro_perdida_mapa = True  
pase_por_mod_manual = True

#declaracion de objetos para publicar
odom_pub = rospy.Publisher("odom_wheels", Odometry, queue_size=50)
br = tf.TransformBroadcaster()
vel_wheel_pub = rospy.Publisher("vel_wheel", Float32MultiArray, queue_size=1)
respuesta_robot = rospy.Publisher("respuesta_robot", Int32MultiArray, queue_size=200)
luces_pub = rospy.Publisher("luces", Int32MultiArray, queue_size=200)

def cb_nano_detector(dato):                  
    global espacio_libre
    if len(dato.grupo_obstaculos)>0:
        for obstaculo in dato.grupo_obstaculos:
            for puntos_array in obstaculo.array_puntos:
                if puntos_array.d < 0.5:
                    espacio_libre = True # cambiar True si la evasion no sirve
                    print("alto por obstaculo cerca")
                else:
                    espacio_libre=True
    else:
        espacio_libre=True

def cb_wheel(data):
    global th_odom
    global x_odom
    global y_odom
    global right_wheel_pose
    global left__wheel_pose
    global dt_ant
    global odom
    global vel_left
    global vel_right

    #rpm a rad/s
    v_right=vel_right
    v_left=vel_left

    #obtiene las velocidades del robot respecto al marco ligado al cuerpo
    v_rx = ((v_right + v_left)*rad_wheels)/2
    v_ry = 0.0
    omega_r = ((v_right - v_left)*rad_wheels) / d_wheels

    #velocidades respecto odom, aplica transformada 2d
    vx_odom = v_rx * math.cos(th_odom)
    vy_odom = v_rx * math.sin(th_odom) 
    vth_odom = omega_r

    #calcula la posicion, hace la integral de la velocidad
    dt = rospy.Time.now().to_sec() - dt_ant
    x_odom = x_odom + vx_odom * dt
    y_odom = y_odom + vy_odom * dt
    th_odom = th_odom + vth_odom * dt

    #calcula la posicion de las ruedas, integra la velocidad angular
    right_wheel_pose = right_wheel_pose + v_right * dt
    left__wheel_pose = left__wheel_pose + v_left * dt
    dt_ant=rospy.Time.now().to_sec()

    #publica transformadas
    q_right_wheel = quaternion_from_euler(1.57,right_wheel_pose,0)
    q_left_wheel = quaternion_from_euler(-1.57,left__wheel_pose,0)
    q_th_odom = quaternion_from_euler(0,0,th_odom)

    #Plubica transformadas
    br.sendTransform((0, -d_wheels/2, 0.0),     q_right_wheel,   rospy.Time.now(),"right_wheel_link","base_link")
    br.sendTransform((0, d_wheels/2, 0.0),      q_left_wheel,    rospy.Time.now(),"left_wheel_link","base_link")
    #br.sendTransform((x_odom,y_odom,0.0), q_th_odom,       rospy.Time.now(),"base_foot_print_wheel","odom")

    #publica odometria
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"
    odom.pose.pose = Pose(Point(x_odom, y_odom, 0.), Quaternion(*q_th_odom))
    odom.child_frame_id = "base_footprint"
    odom.twist.twist = Twist(Vector3(vx_odom, vy_odom, 0), Vector3(0, 0, vth_odom))
    odom_pub.publish(odom)

def cb_twist_vel(data):
    global espacio_libre
    global vel_wheel_msg
    global vel_wheel_pub
    global vel_deseada_th
    global vel_deseada
    global modo_control
    global vel_left
    global vel_right
    global pase_por_mod_manual

    if modo_control==0: #modo manual
        pase_por_mod_manual = True
        vel_left  = ((data.linear.x * k_vel_x - data.angular.z * d_wheels / 2.0)/rad_wheels)
        vel_right = ((data.linear.x * k_vel_x + data.angular.z * d_wheels / 2.0)/rad_wheels)
        vel_wheel_msg.data=[numpy.float32(vel_right),numpy.float32(vel_left)]
        vel_wheel_pub.publish(vel_wheel_msg)

def cb_twist_vel_auto(data):
    global vel_wheel_msg
    global vel_wheel_pub
    global vel_deseada
    global vel_deseada_th
    global modo_control
    global vel_left
    global vel_right
    global paro_perdida_mapa


    if modo_control==1 and paro_perdida_mapa: #modo manual
        vel_left  = ((data.linear.x * k_vel_x  - data.angular.z * d_wheels / 2.0)/rad_wheels)
        vel_right = ((data.linear.x * k_vel_x + data.angular.z * d_wheels / 2.0)/rad_wheels)
        vel_wheel_msg.data=[numpy.float32(vel_right),numpy.float32(vel_left)]
        vel_wheel_pub.publish(vel_wheel_msg)

def cb_set_vel(vel):
    global vel_deseada
    global vel_deseada_th
    vel_deseada = vel.data
    vel_deseada_th = vel.data
   
def cb_estado_robot(dato): 
    global modo_control 
    global senalizacion_robot
    global modo_control_ant
    global pase_por_mod_manual
    respuesta_robot_msg = Int32MultiArray()
    
    respuesta_robot_msg.data = [13,1]
    respuesta_robot.publish(respuesta_robot_msg)

    
    if dato.data[0] == 0: # modo de control
        if dato.data[1] == 0: #modo manual
            modo_control = dato.data[1]
            senalizacion_robot = 0
            print "modo manual"
        elif dato.data[1] == 1: #modo auto
            modo_control = dato.data[1]
            senalizacion_robot = 1
            print "modo auto"

        elif dato.data[1] == 2: #standby
            modo_control = dato.data[1]
            senalizacion_robot = 9
            print "standby"
        respuesta_robot.publish(respuesta_robot_msg)
        while respuesta_robot.get_num_connections() < 1:
            print "esperando a conectarse"
        respuesta_robot.publish(respuesta_robot_msg)
        respuesta_robot.publish(respuesta_robot_msg)
        
    elif dato.data[0] == 1: #activiad del robot
        if paro_perdida_mapa:
            senalizacion_robot = int(dato.data[1])   

    elif dato.data[0] == 2: #paro de emergencia 
        print dato
        if dato.data[1] == 1:
            modo_control = 3
            senalizacion_robot = 9
            print "paro activo"

        if dato.data[1] == 0:
            modo_control = 0
            senalizacion_robot = 0
            print "paro desactivado"

        while respuesta_robot.get_num_connections() < 1:
            print "esperando a conectarse"
        respuesta_robot.publish(respuesta_robot_msg)
    
    elif dato.data[0] == 3: #paro perdida mapa
        if dato.data[1] == 1:
            paro_perdida_mapa = False
            senalizacion_robot = 8
            print "paro activo"
            pase_por_mod_manual = False

        if dato.data[1] == 0 and pase_por_mod_manual:
            paro_perdida_mapa = True
            senalizacion_robot = 0
            print "paro desactivado"

def apagado():
    apagar_nano_msg=Int32()
    apagar_nano_msg.data = 1
    #apagar_nano_pub.publish(apagar_nano_msg)

def publicador_de_odometrias():  
    
    global dt_ant
    global dt_ant_odom 

    rospy.init_node('odom_wheel_node')
    rospy.Subscriber("/wheel_data", Float32MultiArray, cb_wheel,queue_size=30)
    rospy.Subscriber("/cmd_vel", Twist, cb_twist_vel,queue_size=30)
    rospy.Subscriber("/vel_deseada", Float32, cb_set_vel,queue_size=30)
    rospy.Subscriber("/set_estado", Int32MultiArray, cb_estado_robot,queue_size=30)
    rospy.Subscriber("/planner/cmd_vel", Twist, cb_twist_vel_auto,queue_size=30)
    rospy.Subscriber("/pub_grupo_obs", GrupoObstaculos, cb_nano_detector,queue_size=100)
    #rospy.Subscriber("/zed/odom", Odometry,cb_GetRobotOdom, queue_size=50)    
    dt_ant=rospy.Time.now().to_sec()
    rospy.on_shutdown(apagado)
    rate = rospy.Rate(2)
    encendido = False
    while not rospy.is_shutdown():
        if senalizacion_robot == 0: # manual
        
            if encendido:
                luces_msg.data = [1,0,0,0]
                luces_pub.publish(luces_msg)
            else:
                luces_msg.data = [1,0,0,0]
                luces_pub.publish(luces_msg)
            
        elif senalizacion_robot == 1: # auto sin botella 
            if encendido:
                luces_msg.data = [1,0,0,0]
                luces_pub.publish(luces_msg)
            else:
                luces_msg.data = [0,0,0,0]
                luces_pub.publish(luces_msg)
        
        elif senalizacion_robot == 2: # auto con botella
            if encendido:
                luces_msg.data = [1,0,0,0]
                luces_pub.publish(luces_msg)
            else:
                luces_msg.data = [0,1,0,0]
                luces_pub.publish(luces_msg)
        
        elif senalizacion_robot == 3: # auto reversa
            if encendido:
                luces_msg.data = [0,0,0,0]
                luces_pub.publish(luces_msg)
            else:
                luces_msg.data = [0,1,0,1]
                luces_pub.publish(luces_msg)
        
        elif senalizacion_robot == 4: # auto espera botella
            if encendido:
                luces_msg.data = [1,1,0,0]
                luces_pub.publish(luces_msg)
            else:
                luces_msg.data = [0,0,0,0]
                luces_pub.publish(luces_msg)

        elif senalizacion_robot == 5: # bateria baja
            if encendido:
                luces_msg.data = [0,0,0,0]
                luces_pub.publish(luces_msg)
            else:
                luces_msg.data = [0,0,1,1]
                luces_pub.publish(luces_msg)

        elif senalizacion_robot == 6: # obstaculo atravesado
            if encendido:
                luces_msg.data = [0,1,0,1]
                luces_pub.publish(luces_msg)
            else:
                luces_msg.data = [0,0,0,0]
                luces_pub.publish(luces_msg)

        elif senalizacion_robot == 7: # apunto de irse de maquina
            if encendido:
                luces_msg.data = [1,1,0,1]
                luces_pub.publish(luces_msg)
            else:
                luces_msg.data = [0,0,0,0]
                luces_pub.publish(luces_msg)

        elif senalizacion_robot == 8: # error
            if encendido:
                luces_msg.data = [0,0,1,1]
                luces_pub.publish(luces_msg)
            else:
                luces_msg.data = [0,0,1,0]
                luces_pub.publish(luces_msg)

        elif senalizacion_robot == 9: # paro de emrgencia
            if encendido:
                luces_msg.data = [0,0,1,0]
                luces_pub.publish(luces_msg)
            else:
                luces_msg.data = [0,0,0,0]
                luces_pub.publish(luces_msg)

        if encendido:
            encendido = False
        else:
            encendido = True
        
        rate.sleep()

if __name__ == '__main__':
    
    publicador_de_odometrias()
