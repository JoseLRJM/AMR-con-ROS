#!/usr/bin/env python  

import roslib
import rospy
import numpy as np
import tf
import yaml
import os.path
import math
import roslaunch
import time
from os import path
import dynamic_reconfigure.client
from std_msgs.msg import Int32,UInt32MultiArray,Int32MultiArray,String,Bool,Float32MultiArray,Int16MultiArray
from geometry_msgs.msg import Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from variables_mision import templador,maquina

hay_mision_cargada=False
slam_lanzado = False
modo_auto_activo = False
altura_stacker = 0.0

templadores = templador()
maquinas = maquina()

lista_archivos = rospy.Publisher("list_request", String, queue_size=100)
lista_archivos_misiones = rospy.Publisher("list_request_misiones", String, queue_size=100)

marker_data = rospy.Publisher("marker_data", Float32MultiArray, queue_size=1)
misiones = rospy.Publisher("load_mission", Float32MultiArray, queue_size=200)
respuesta_robot = rospy.Publisher("respuesta_robot", Int32MultiArray, queue_size=200)
# 0 repsuesta para lanzamiento del mapeo
# 1 respuesta para terminar mapeo

def cb_camera_params(dyna_params):
    zed2.update_configuration({"brightness":  dyna_params.data[0],
                                "contrast":dyna_params.data[1],
                                "hue":dyna_params.data[2],
                                "saturation":dyna_params.data[3],
                                "sharpness": dyna_params.data[4],
                                "gamma": dyna_params.data[5],
                                "gain": dyna_params.data[6],
                                "exposure": dyna_params.data[7]})

def cb_parametros(dyna_params):

    id_params = int(dyna_params.data[0])
    if id_params == 10:
        zed2.update_configuration({"brightness":  int(dyna_params.data[1])})
    elif id_params == 11:
        zed2.update_configuration({"contrast":  int(dyna_params.data[1])})
    elif id_params == 12:
        zed2.update_configuration({"auto_exposure_gain":  bool(dyna_params.data[1])})
    elif id_params == 13:
        zed2.update_configuration({"gain":  int(dyna_params.data[1])})
    elif id_params == 14:
        zed2.update_configuration({"exposure":  int(dyna_params.data[1])})
    
    zed2.update_configuration({"whitebalance_temperature":  40})
    #zed2.update_configuration({"whitebalance_temperature":  40})

def cb_marker_data(dato):
    global slam_lanzado
    global modo_auto_activo
    if dato.markers:
        for marker_odom_msg in dato.markers:
            if marker_odom_msg.id < 30 and marker_odom_msg.id > 0:
                marker_msg = Float32MultiArray()

                quaternion = (marker_odom_msg.pose.pose.orientation.x, marker_odom_msg.pose.pose.orientation.y, marker_odom_msg.pose.pose.orientation.z, marker_odom_msg.pose.pose.orientation.w)
                theta = tf.transformations.euler_from_quaternion(quaternion)[2]

                mark_pos_theta = theta + np.pi / 2.

                if abs(marker_odom_msg.pose.pose.position.y) < 0.3:
                    if abs(mark_pos_theta) < 0.7:
                        marker_msg.data.append(marker_odom_msg.pose.pose.position.x)
                        marker_msg.data.append(mark_pos_theta)
                        marker_msg.data.append(marker_odom_msg.id)
                        marker_msg.data.append(altura_stacker)
                        marker_msg.data.append(-marker_odom_msg.pose.pose.position.y)
                        marker_data.publish(marker_msg)

def cb_publicador_app(dato):
    global slam_lanzado
    global modo_auto_activo
    respuesta_robot_msg = Int32MultiArray()
    #print dato.data

    if dato.data == 1: #envio lista de mapas
        print "lista de mapas"
        lista_env = String()
        lista = os.listdir("/home/user/mapas")
        lista_env.data = ",".join(lista)
        time.sleep(0.3)
        while lista_archivos.get_num_connections() < 1:
            print "esperando a conectarse"
        lista_archivos.publish(lista_env)
        respuesta_robot_msg.data = [11,1]
        #respuesta_robot.publish(respuesta_robot_msg)
        print lista_env
           
    elif dato.data == 2: #envio lista de misiones
        print "lista de mapas"
        lista_en = String()
        lista = os.listdir("/home/user/misiones")
        lista_en.data = ",".join(lista)
        time.sleep(0.3)
        while lista_archivos_misiones.get_num_connections() < 1:
            print "esperando a conectarse"
        lista_archivos_misiones.publish(lista_en)
        respuesta_robot_msg.data = [12,1]
        #respuesta_robot.publish(respuesta_robot_msg)
        print lista_en

    elif dato.data == 5: #matar slam 

        slam_lanzado=False
        print "deteniendo slam localizacion"
        os.system("rosnode kill rtabmap")
        os.system("rosnode kill robot_pose_publisher")
        time.sleep(0.3)
        respuesta_robot_msg.data = [1,1]
        while respuesta_robot.get_num_connections() < 1:
            print "esperando a conectarse"
        respuesta_robot.publish(respuesta_robot_msg)
    
    elif dato.data == 7: #matar modo auto 

        print "deteniendo modo auto"
        os.system("rosparam delete /mision")
        os.system("rosnode kill /mod_auto")
        os.system("rosnode kill /planner/move_base")
        os.system("rosnode kill /Parking_server_node")
        time.sleep(0.3)
        respuesta_robot_msg.data = [10,1]
        while respuesta_robot.get_num_connections() < 1:
            print "esperando a conectarse"
        respuesta_robot.publish(respuesta_robot_msg)

def cb_request_mission(dato):    
    templadores_ = templador()
    maquinas_ = maquina()
    mision_msg = Float32MultiArray()
    if path.exists('/home/user/misiones/'+ dato.data):  # enviar templadores y maquinas
        time.sleep(0.3)
        with open(r'/home/user/misiones/'+ dato.data) as file:
            print("entre al archivo")
            mision_data = yaml.load(file)

            templadores_.num =           mision_data["datos"]["numero_templadores"]
            maquinas_.num =              mision_data["datos"]["numero_maquinas"]
            
            if templadores_.num > 0 and maquinas_.num > 0:
                
                templadores_.ID =            mision_data["templadores"]["ID"]
                templadores_.x =             mision_data["templadores"]["x"]
                templadores_.y =             mision_data["templadores"]["y"]
                templadores_.qz =            mision_data["templadores"]["qz"]
                templadores_.qw =            mision_data["templadores"]["qw"]
                templadores_.tagPosX =       mision_data["templadores"]["tagPosX"]
                templadores_.tagAng =        mision_data["templadores"]["tagAng"]
                templadores_.tag_id =        mision_data["templadores"]["tag_id"]
                templadores_.altura =        mision_data["templadores"]["altura"]
                templadores_.tagPosY =       mision_data["templadores"]["tagPosY"]

                maquinas_.ID =               mision_data["maquinas"]["ID"]
                maquinas_.x =                mision_data["maquinas"]["x"]
                maquinas_.y =                mision_data["maquinas"]["y"]
                maquinas_.qz =               mision_data["maquinas"]["qz"]
                maquinas_.qw =               mision_data["maquinas"]["qw"]
                maquinas_.periodo =          mision_data["maquinas"]["periodo"]
                maquinas_.vel_botella =      mision_data["maquinas"]["vel_botella"]
                maquinas_.vel_stacker =      mision_data["maquinas"]["vel_stacker"]
                maquinas_.templador_id =     mision_data["maquinas"]["templador_id"]
                maquinas_.accion_id =        mision_data["maquinas"]["accion_id"]
                maquinas_.accion_boton_id =  mision_data["maquinas"]["accion_boton_id"]
                maquinas_.tagPosX =          mision_data["maquinas"]["tagPosX"]
                maquinas_.tagAng =           mision_data["maquinas"]["tagAng"]
                maquinas_.tag_id =           mision_data["maquinas"]["tag_id"]
                maquinas_.altura =           mision_data["maquinas"]["altura"]
                maquinas_.tagPosY =          mision_data["maquinas"]["tagPosY"]
                maquinas_.tiempoAlerta =     mision_data["maquinas"]["tiempoAlerta"]
                maquinas_.tiempoEspera =     mision_data["maquinas"]["tiempoEspera"]
                
                mision_msg.data.append(templadores_.num)
                mision_msg.data.append(maquinas_.num)
                #print maquinas_.num
                for x in range(0, templadores_.num):
                    mision_msg.data.append(1) 
                    mision_msg.data.append(templadores_.ID[x])
                    mision_msg.data.append(templadores_.x[x])
                    mision_msg.data.append(templadores_.y[x])
                    mision_msg.data.append(templadores_.qz[x])
                    mision_msg.data.append(templadores_.qw[x])
                    mision_msg.data.append(templadores_.tagPosX[x])
                    mision_msg.data.append(templadores_.tagAng[x])
                    mision_msg.data.append(templadores_.tag_id[x])
                    mision_msg.data.append(templadores_.altura[x])
                    mision_msg.data.append(templadores_.tagPosY[x])

                for x in range(0, maquinas_.num):
                    mision_msg.data.append(2)
                    mision_msg.data.append(maquinas_.ID[x])
                    mision_msg.data.append(maquinas_.x[x])
                    mision_msg.data.append(maquinas_.y[x])
                    mision_msg.data.append(maquinas_.qz[x])
                    mision_msg.data.append(maquinas_.qw[x])
                    mision_msg.data.append(maquinas_.periodo[x])
                    mision_msg.data.append(maquinas_.vel_botella[x])
                    mision_msg.data.append(maquinas_.vel_stacker[x])
                    mision_msg.data.append(maquinas_.templador_id[x])
                    mision_msg.data.append(maquinas_.accion_id[x])
                    mision_msg.data.append(maquinas_.accion_boton_id[x]) 
                    mision_msg.data.append(maquinas_.tagPosX[x]) 
                    mision_msg.data.append(maquinas_.tagAng[x]) 
                    mision_msg.data.append(maquinas_.tag_id[x]) 
                    mision_msg.data.append(maquinas_.altura[x])
                    mision_msg.data.append(maquinas_.tagPosY[x])
                    mision_msg.data.append(maquinas_.tiempoAlerta[x])
                    mision_msg.data.append(maquinas_.tiempoEspera[x])


                misiones.publish(mision_msg)
                print("MISION ENVIADA")

            else:
                mision_msg.data.append(0)
                mision_msg.data.append(0)
                misiones.publish(mision_msg)
                print("ERROR NO HAY MISIONES")

def cb_set_mis_ion(data):
    global templadores
    global maquinas
    respuesta_robot_msg = Int32MultiArray()
    while respuesta_robot.get_num_connections() < 1:
            print "esperando a conectarse"
    respuesta_robot_msg.data = [2,1]
    respuesta_robot.publish(respuesta_robot_msg)
    
    # agregar templador
    if int(data.data[0])==1:

        index_templador=                            int(data.data[1])
        templadores.ID[index_templador]=            index_templador
        templadores.x[index_templador]=             data.data[2]
        templadores.y[index_templador]=             data.data[3]
        templadores.qz[index_templador]=            data.data[4]
        templadores.qw[index_templador]=            data.data[5]
        templadores.tag_id[index_templador]=        int(data.data[6])
        templadores.tagPosX[index_templador]=   data.data[7]
        templadores.tagAng[index_templador]=   data.data[8]
        templadores.altura[index_templador]=        data.data[9]
        templadores.tagPosY[index_templador]=       data.data[10]

        print ("llego templador")
        rospy.set_param("mision/templadores", {"ID": templadores.ID,
                                                "x": templadores.x,
                                                "y": templadores.y,
                                                "qz": templadores.qz,
                                                "qw": templadores.qw,
                                                "tagPosX": templadores.tagPosX,
                                                "tagAng": templadores.tagAng,
                                                "tag_id": templadores.tag_id,
                                                "altura": templadores.altura,
                                                "tagPosY": templadores.tagPosY})
    # agregar maquina
    elif int(data.data[0])==2:
        print ("llego maquina")
        numero_maquina=                          int(data.data[1])
        maquinas.ID[numero_maquina]=             numero_maquina
        maquinas.x[numero_maquina]=              data.data[2]
        maquinas.y[numero_maquina]=              data.data[3]
        maquinas.qz[numero_maquina]=             data.data[4]
        maquinas.qw[numero_maquina]=             data.data[5]
        maquinas.periodo[numero_maquina]=        data.data[6]
        maquinas.vel_botella[numero_maquina]=    data.data[7]
        maquinas.vel_stacker[numero_maquina]=    data.data[8]
        maquinas.templador_id[numero_maquina]=   int(data.data[9])
        maquinas.accion_id[numero_maquina]=      int(data.data[10]) #0=maquina tag 1=aquina_operador
        maquinas.accion_boton_id[numero_maquina]=int(data.data[11]) 
        maquinas.tagPosX[numero_maquina]=        data.data[12]  #id_tag
        maquinas.tagAng[numero_maquina]=         data.data[13]  #angulo
        maquinas.tag_id[numero_maquina]=         data.data[14]  #distancia
        maquinas.altura[numero_maquina]=         data.data[16]  
        maquinas.tagPosY[numero_maquina]=        data.data[17]
        maquinas.tiempoAlerta[numero_maquina]=   data.data[18]
        maquinas.tiempoEspera[numero_maquina]=   data.data[19]

        rospy.set_param("mision/maquinas", {"ID": maquinas.ID,"x": maquinas.x,
                                            "y": maquinas.y,
                                            "qz": maquinas.qz,
                                            "qw": maquinas.qw,
                                            "periodo": maquinas.periodo,
                                            "vel_botella": maquinas.vel_botella,
                                            "vel_stacker": maquinas.vel_stacker,
                                            "accion_id": maquinas.accion_id,
                                            "templador_id": maquinas.templador_id,
                                            "accion_boton_id": maquinas.accion_boton_id,
                                            "tagPosX": maquinas.tagPosX,
                                            "tagAng": maquinas.tagAng,
                                            "tag_id": maquinas.tag_id,
                                            "altura": maquinas.altura,
                                            "tagPosY": maquinas.tagPosY,
                                            "tiempoAlerta": maquinas.tiempoAlerta,
                                            "tiempoEspera": maquinas.tiempoEspera })
    #longitud de la mision
    elif int(data.data[0])==0:
        templadores.num=int(data.data[1])
        maquinas.num=int(data.data[2])
        rospy.set_param("mision/datos", {"numero_templadores": templadores.num,
                                        "numero_maquinas": maquinas.num})
        print(data.data)
    #eliminar templador
    elif int(data.data[0])==3:
        numero_templador_eliminar=int(data.data[1])
        templadores.ID.pop(numero_templador_eliminar)
        templadores.x.pop(numero_templador_eliminar)
        templadores.y.pop(numero_templador_eliminar)
        templadores.qz.pop(numero_templador_eliminar)
        templadores.qw.pop(numero_templador_eliminar)
        templadores.tagPosX.pop(numero_templador_eliminar)
        templadores.tagAng.pop(numero_templador_eliminar)
        templadores.tag_id.pop(numero_templador_eliminar)
        templadores.altura.pop(numero_templador_eliminar)
        templadores.tagPosY.pop(numero_templador_eliminar)

        rospy.set_param("mision/templadores", {"ID": templadores.ID,
                                                "x": templadores.x,
                                                "y": templadores.y,
                                                "qz": templadores.qz,
                                                "qw": templadores.qw,
                                                "tagPosX": templadores.tagPosX,
                                                "tagAng": templadores.tagAng,
                                                "tag_id": templadores.tag_id,
                                                "altura": templadores.altura,
                                                "tagPosY": templadores.tagPosY})
        if hay_mision_cargada:
            templadores.num = templadores.num-1
            maquinas.num = maquinas.num-1
            rospy.set_param("mision/datos", {"numero_templadores": templadores.num,
                                            "numero_maquinas": maquinas.num })
        print ("elimino templador")
    #eliminar maquina   
    elif int(data.data[0])==4:
        
        numero_maquina_eliminar=int(data.data[1])
        maquinas.ID.pop(numero_maquina_eliminar)
        maquinas.x.pop(numero_maquina_eliminar)
        maquinas.y.pop(numero_maquina_eliminar)
        maquinas.qz.pop(numero_maquina_eliminar)
        maquinas.qw.pop(numero_maquina_eliminar)
        maquinas.periodo.pop(numero_maquina_eliminar)
        maquinas.vel_botella.pop(numero_maquina_eliminar)
        maquinas.vel_stacker.pop(numero_maquina_eliminar)
        maquinas.templador_id.pop(numero_maquina_eliminar)
        maquinas.accion_id.pop(numero_maquina_eliminar)
        maquinas.accion_boton_id.pop(numero_maquina_eliminar)
        maquinas.tagPosX.pop(numero_maquina_eliminar)
        maquinas.tagAng.pop(numero_maquina_eliminar)
        maquinas.tag_id.pop(numero_maquina_eliminar)
        maquinas.altura.pop(numero_maquina_eliminar)
        maquinas.tagPosY.pop(numero_maquina_eliminar)
        maquinas.tiempoAlerta.pop(numero_maquina_eliminar)
        maquinas.tiempoEspera.pop(numero_maquina_eliminar)

        rospy.set_param("mision/maquinas", {"ID": maquinas.ID,
                                            "x": maquinas.x,
                                            "y": maquinas.y,
                                            "qz": maquinas.qz,
                                            "qw": maquinas.qw,
                                            "periodo": maquinas.periodo,
                                            "vel_botella": maquinas.vel_botella,
                                            "vel_stacker": maquinas.vel_stacker,
                                            "accion_id": maquinas.accion_id,
                                            "templador_id": maquinas.templador_id,
                                            "accion_boton_id": maquinas.accion_boton_id,
                                            "tagPosX": maquinas.tagPosX,
                                            "tagAng": maquinas.tagAng,
                                            "tag_id": maquinas.tag_id,
                                            "altura": maquinas.altura,
                                            "tagPosY": maquinas.tagPosY,
                                            "tiempoAlerta": maquinas.tiempoAlerta,
                                            "tiempoEspera": maquinas.tiempoEspera })
        if hay_mision_cargada:
            templadores.num = templadores.num-1
            maquinas.num = maquinas.num-1
            rospy.set_param("mision/datos", {"numero_templadores": templadores.num,
                                            "numero_maquinas": maquinas.num })
        print ("elimino maquina")

def cb_set_mision(data):
    global templadores
    global maquinas
    respuesta_robot_msg = Int32MultiArray()
    while respuesta_robot.get_num_connections() < 1:
            print "esperando a conectarse"
    respuesta_robot_msg.data = [2,1]
    respuesta_robot.publish(respuesta_robot_msg)

    templadores.num=int(data.data[0])
    maquinas.num=int(data.data[1])

    rospy.set_param("mision/datos", {"numero_templadores": templadores.num,
                                     "numero_maquinas": maquinas.num})
    next_id = 0
    
    for i in range(0,templadores.num):
        if data.data[i*11+2] == 1:
            templadores.ID[i] =             data.data[i*11+3]
            templadores.x[i] =              data.data[i*11+4]
            templadores.y[i] =              data.data[i*11+5]
            templadores.qz[i] =             data.data[i*11+6]
            templadores.qw[i] =             data.data[i*11+7]
            templadores.tagPosX[i] =        data.data[i*11+8]
            templadores.tagAng[i] =         data.data[i*11+9]
            templadores.tag_id[i] =         data.data[i*11+10]
            templadores.altura[i] =         data.data[i*11+11]
            templadores.tagPosY[i] =        data.data[i*11+12]
        next_id = i*11+13

    for i_maquina in range(0,maquinas.num):
        if data.data[i_maquina*19 + next_id] == 2:

            maquinas.ID[i_maquina]=             data.data[i_maquina*19 + next_id + 1]
            maquinas.x[i_maquina]=              data.data[i_maquina*19 + next_id + 2]
            maquinas.y[i_maquina]=              data.data[i_maquina*19 + next_id + 3]
            maquinas.qz[i_maquina]=             data.data[i_maquina*19 + next_id + 4]
            maquinas.qw[i_maquina]=             data.data[i_maquina*19 + next_id + 5]
            maquinas.periodo[i_maquina]=        data.data[i_maquina*19 + next_id + 6]
            maquinas.vel_botella[i_maquina]=    data.data[i_maquina*19 + next_id + 7]
            maquinas.vel_stacker[i_maquina]=    data.data[i_maquina*19 + next_id + 8]
            maquinas.templador_id[i_maquina]=   int(data.data[i_maquina*19 + next_id + 9])
            maquinas.accion_id[i_maquina]=      int(data.data[i_maquina*19 + next_id + 10])
            maquinas.accion_boton_id[i_maquina]=int(data.data[i_maquina*19 + next_id + 11]) 
            maquinas.tagPosX[i_maquina]=        data.data[i_maquina*19 + next_id + 12]  
            maquinas.tagAng[i_maquina]=         data.data[i_maquina*19 + next_id + 13]  
            maquinas.tag_id[i_maquina]=         data.data[i_maquina*19 + next_id + 14] 
            maquinas.altura[i_maquina]=         data.data[i_maquina*19 + next_id + 15]  
            maquinas.tagPosY[i_maquina]=        data.data[i_maquina*19 + next_id + 16]
            maquinas.tiempoAlerta[i_maquina]=   data.data[i_maquina*19 + next_id + 17]
            maquinas.tiempoEspera[i_maquina]=   data.data[i_maquina*19 + next_id + 18]

    rospy.set_param("mision/templadores", {"ID": templadores.ID,
                                                "x": templadores.x,
                                                "y": templadores.y,
                                                "qz": templadores.qz,
                                                "qw": templadores.qw,
                                                "tagPosX": templadores.tagPosX,
                                                "tagAng": templadores.tagAng,
                                                "tag_id": templadores.tag_id,
                                                "altura": templadores.altura,
                                                "tagPosY": templadores.tagPosY})

    rospy.set_param("mision/maquinas", {"ID": maquinas.ID,"x": maquinas.x,
                                        "y": maquinas.y,
                                        "qz": maquinas.qz,
                                        "qw": maquinas.qw,
                                        "periodo": maquinas.periodo,
                                        "vel_botella": maquinas.vel_botella,
                                        "vel_stacker": maquinas.vel_stacker,
                                        "accion_id": maquinas.accion_id,
                                        "templador_id": maquinas.templador_id,
                                        "accion_boton_id": maquinas.accion_boton_id,
                                        "tagPosX": maquinas.tagPosX,
                                        "tagAng": maquinas.tagAng,
                                        "tag_id": maquinas.tag_id,
                                        "altura": maquinas.altura,
                                        "tagPosY": maquinas.tagPosY,
                                        "tiempoAlerta": maquinas.tiempoAlerta,
                                        "tiempoEspera": maquinas.tiempoEspera })


if __name__ == "__main__":
    rospy.init_node("envio_app")
    zed2 = dynamic_reconfigure.client.Client("/zed")

    rospy.Subscriber("/publicador_app", Int32, cb_publicador_app,queue_size=100)
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers,cb_marker_data,  queue_size=100)
    rospy.Subscriber("/publicador_mision", String,cb_request_mission,  queue_size=100)
    rospy.Subscriber("/camera_params", Int32MultiArray, cb_camera_params, queue_size=1)
    rospy.Subscriber("/set_mission", Float32MultiArray, cb_set_mision,queue_size=100)
    rospy.Subscriber("/parametros", Float32MultiArray, cb_parametros, queue_size=100)

    rospy.spin() 


