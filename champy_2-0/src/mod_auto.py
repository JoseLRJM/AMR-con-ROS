#!/usr/bin/env python  
from variables_mision import templador,maquina
from enum import Enum
import roslib
import rospy
import actionlib
import math
import time
import dynamic_reconfigure.client
from actionlib import SimpleActionClient, GoalStatus
from std_srvs.srv import Empty as Empty_msg_srv, EmptyRequest
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
from nav_msgs.srv import GetPlan, GetPlanRequest
from std_msgs.msg import Float32MultiArray,Float32,Int8MultiArray,Int32MultiArray,Int8,Int32,Empty
from champy_action_msgs.msg import parkingGoal,parkingAction, parkingFeedback, parkingResult
from champy_action_msgs.msg import unparkingGoal,unparkingAction, unparkingFeedback, unparkingResult
from geometry_msgs.msg import Quaternion, PoseStamped, Twist

time.sleep(1)

reversa_habilitada = False
tiempo_transcurrido_reversa = 0.0
tiempo_de_reversa_deseado = 2.0
tiempo_inicial_en_reversa = 0.0

templadores=templador()
maquinas=maquina()
goal_d = True
actuadores_gpu_msg = Int32MultiArray()
set_estado_msg = Int32MultiArray()
plc_msg = Int32MultiArray()
estado_robot_msg = Int8()
vel_deseada_msg = Float32()

set_estado_msg.data = [0,0]
plc_msg.data = [0,101]
maquinas_periodo_curent = [0]
vel_sin_botella = 2.5
margen_de_arribo = 5
numero_maquina = 0

#variables globales de goals
status_viaje=0
past_core_fase=-1
maquina_ant = -1
next_maquina=-1
next_templador=0
pos_x=0
pos_y=0
hay_mision_cargada = False
tiempo_transcurrido = 0 
tiempo_transcurrido_anterior = time.time()
BOTONES_ACCION_MAQUINA=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
boton_pre_id = 0
boton_pre_estado = 0

#estado del robot
porcentaje_bateria = 0.0
voltaje = 0.0
corriente = 0.0
pos_stacker = 2
estado_sujetador_botellas = 2
pos_sujetador_botellas = 0.0
paro_emergencia = 0

#parametros de navegacion
vel_meter_stacker = 70
velocidad_a_maquina = 0.8
max_vel_nav = 1.55
escapevel = 0.0 #-0.1
distancia_desestacionado = 1.0

tiempo_inicio_boton=0
tiempo_transcurrido_boton=0
dejando_maquina_tiempo_limite = False

actuadores_gpu_pub = rospy.Publisher("actuadores_gpu", Int32MultiArray, queue_size=1)
set_estado_pub = rospy.Publisher("set_estado", Int32MultiArray, queue_size=1)
estado_robot_pub = rospy.Publisher("estado_robot", Int8, queue_size=1)
pub_cmd_vel = rospy.Publisher("/planner/cmd_vel", Twist, queue_size=1)
plc_senal_pub = rospy.Publisher("/PLC", Int32MultiArray, queue_size=1)

twist = Twist()
twist.linear.x = 0
twist.linear.y = 0
twist.linear.z = 0
twist.angular.x = 0
twist.angular.y = 0
twist.angular.z = 0

def get_param():
    global hay_mision_cargada
    global maquinas
    global templadores

    templadores.num =           rospy.get_param('mision/datos/numero_templadores', None)
    maquinas.num =              rospy.get_param('mision/datos/numero_maquinas', None)
    global numero_maquina

    if not templadores.num == None and not maquinas.num == None:
        
        templadores.ID =            rospy.get_param('mision/templadores/ID', 0)
        templadores.x =             rospy.get_param('mision/templadores/x', 0)
        templadores.y =             rospy.get_param('mision/templadores/y', 0)
        templadores.qz =            rospy.get_param('mision/templadores/qz', 0)
        templadores.qw =            rospy.get_param('mision/templadores/qw', 0)
        templadores.tagPosX =       rospy.get_param('mision/templadores/tagPosX', 0)
        templadores.tagAng =        rospy.get_param('mision/templadores/tagAng', 0)
        templadores.tag_id =        rospy.get_param('mision/templadores/tag_id', 0)
        templadores.altura =        rospy.get_param('mision/templadores/altura', 0)
        templadores.tagPosY =       rospy.get_param('mision/templadores/tagPosY', 0)

        maquinas.ID =               rospy.get_param('mision/maquinas/ID', 0)
        maquinas.x =                rospy.get_param('mision/maquinas/x', 0)
        maquinas.y =                rospy.get_param('mision/maquinas/y', 0)
        maquinas.qz =               rospy.get_param('mision/maquinas/qz', 0)
        maquinas.qw =               rospy.get_param('mision/maquinas/qw', 0)
        maquinas.periodo =          rospy.get_param('mision/maquinas/periodo', 0)
        maquinas.vel_botella =      rospy.get_param('mision/maquinas/vel_botella', 0)
        maquinas.vel_stacker =      rospy.get_param('mision/maquinas/vel_stacker', 0)
        maquinas.templador_id =     rospy.get_param('mision/maquinas/templador_id', 0)
        maquinas.accion_id =        rospy.get_param('mision/maquinas/accion_id', 0)
        maquinas.accion_boton_id =  rospy.get_param('mision/maquinas/accion_boton_id', 0)
        maquinas.tagPosX =          rospy.get_param('mision/maquinas/tagPosX', 0) #distancia
        maquinas.tagAng =           rospy.get_param('mision/maquinas/tagAng', 0) #angulo
        maquinas.tag_id =           rospy.get_param('mision/maquinas/tag_id', 0) #id tag
        maquinas.altura =           rospy.get_param('mision/maquinas/altura', 0)
        maquinas.tagPosY =          rospy.get_param('mision/maquinas/tagPosY', 0)
        maquinas.tiempoAlerta =     rospy.get_param('mision/maquinas/tiempoAlerta', 0)
        maquinas.tiempoEspera =     rospy.get_param('mision/maquinas/tiempoEspera', 0)

        numero_templador = templadores.num -1
        numero_maquina = maquinas.num -1

        hay_mision_cargada=True
        print(maquinas)
        print(templadores)
        
def mover_actuadores(id,pose,vel):
    actuadores_gpu_msg.data=[id,pose,vel]
    actuadores_gpu_pub.publish(actuadores_gpu_msg)
    time.sleep(0.2)
    actuadores_gpu_pub.publish(actuadores_gpu_msg)
    time.sleep(0.2)
    actuadores_gpu_pub.publish(actuadores_gpu_msg)

def cb_data_robot(dato):
    global porcentaje_bateria
    global voltaje
    global corriente
    global pos_stacker 
    global estado_sujetador_botellas
    global pos_sujetador_botellas
    global paro_emergencia

    porcentaje_bateria = dato.data[0]
    voltaje = dato.data[1]
    corriente = dato.data[2]
    pos_stacker = int(dato.data[3])
    estado_sujetador_botellas = int(dato.data[4])
    pos_sujetador_botellas = dato.data[5]
    paro_emergencia = int(dato.data[8])

def cb_BOTON(dato):
    global boton_pre_id
    global boton_pre_estado
    boton_pre_id=int(dato.data[1]) 
    boton_pre_estado=int(dato.data[0])

def conteo_tiempo():
    global maquinas_periodo_curent
    global hay_mision_cargada
    global tiempo_transcurrido
    global tiempo_transcurrido_anterior
    global numero_maquina
    
    tiempo_transcurrido = time.time() - tiempo_transcurrido_anterior
    if hay_mision_cargada:
        for i in range(0,numero_maquina+1):
            if maquinas_periodo_curent[i] < -10:
                maquinas.periodo = rospy.get_param('mision/maquinas/periodo', 0)
                maquinas_periodo_curent[i] = maquinas.periodo[i]

            maquinas_periodo_curent[i] = maquinas_periodo_curent[i]- tiempo_transcurrido
    tiempo_transcurrido_anterior=time.time()

def calcular_tiempo_trayecto(vel,x_ini, y_ini, x_fin, y_fin):
    start = PoseStamped()
    qz_ini = 0
    qw_ini = 1
    qz_fin = 0
    qw_fin = 1
    start.header.seq = 0
    start.header.frame_id = "map"
    start.header.stamp = rospy.Time.now()
    start.pose.position.x = x_ini
    start.pose.position.y = y_ini
    start.pose.orientation = Quaternion(0,0,qz_ini,qw_ini)

    Goal = PoseStamped()
    Goal.header.seq = 0
    Goal.header.frame_id = "map"
    Goal.header.stamp = rospy.Time.now()
    Goal.pose.position.x = x_fin
    Goal.pose.position.y = y_fin
    Goal.pose.orientation = Quaternion(0,0,qz_fin,qw_fin)

    plan_sim = GetPlan()
    plan_sim.start = start
    plan_sim.goal = Goal
    plan_sim.tolerance = 0.1
    
    plan_simulado = get_plan(plan_sim.start, plan_sim.goal, plan_sim.tolerance)
    #print(plan_simulado.plan.poses[0].pose.position)
    distancia_total=0.0
    for i in range(0,len(plan_simulado.plan.poses)-1):
        dt_distancia = math.sqrt((plan_simulado.plan.poses[i+1].pose.position.x-plan_simulado.plan.poses[i].pose.position.x)**2+(plan_simulado.plan.poses[i+1].pose.position.y-plan_simulado.plan.poses[i].pose.position.y)**2)
        distancia_total = distancia_total + dt_distancia
    tiempo=(distancia_total/vel)+5
    
    return tiempo
    print tiempo
    
def cb_pose(dato):
    global pos_x
    global pos_y
    pos_x = dato.pose.position.x
    pos_y = dato.pose.position.y

def activar_reversa():
    global reversa_habilitada
    global tiempo_transcurrido_reversa
    global tiempo_de_reversa_deseado
    global tiempo_inicial_en_reversa
    if reversa_habilitada:            
        tiempo_transcurrido_reversa = time.time() - tiempo_inicial_en_reversa
        if tiempo_transcurrido_reversa > tiempo_de_reversa_deseado:
            set_vel.update_configuration({"min_vel_x": 0.0})
            print "cambiando velocidad a 0"
            reversa_habilitada = False

def cb_parametros(dyna_params):

    id_params = int(dyna_params.data[0])

    if id_params == 0:
        planner.update_configuration({"xy_goal_tolerance":  dyna_params.data[1]})
    elif id_params == 1:
        planner.update_configuration({"yaw_goal_tolerance":  dyna_params.data[1]})
    elif id_params == 2:
        planner.update_configuration({"path_distance_bias":  dyna_params.data[1]})
    elif id_params == 3:
        planner.update_configuration({"goal_distance_bias":  dyna_params.data[1]})
    elif id_params == 4:
        planner.update_configuration({"occdist_scale":  dyna_params.data[1]})
    elif id_params == 5:
        planner.update_configuration({"forward_point_distance":  dyna_params.data[1]})
    
    elif id_params == 6:
        plannerglobal.update_configuration({"neutral_cost":  int(dyna_params.data[1])})  

if __name__ == "__main__":
    
    rospy.init_node("mod_auto")  
      
    r = rospy.Rate(5)
    
    core_fase = Enum('core_fase', 'CARGAR_RUTA STANDBY GO_MAQUINA GO_TEMPLADOR TRAVELING_TO_MAQUINA TRAVELING_TO_TEMPLADOR DO_ACCION_MAQUINA ENTREGA NEXT_MAQUINA')
    recoger_botella = Enum('recoger_botella','SEND_PARKING ESPERA_PARKING ESPERA_BOTON SEND_UNPARKING ESPERA_UNPARKING')
    entrega_botella = Enum('entrega_botella','SEND_PARKING ESPERA_PARKING ESPERA_STACKER SEND_UNPARKING ESPERA_UNPARKING')
    
    get_param()
    current_recoger_botella_fase = recoger_botella.SEND_PARKING.value
    current_entrega_botella_fase = entrega_botella.SEND_PARKING.value
    current_core_fase = core_fase.CARGAR_RUTA.value
    
    rospy.Subscriber("data_robot", Float32MultiArray, cb_data_robot,queue_size=100)
    rospy.Subscriber("BOTON", Float32MultiArray, cb_BOTON,queue_size=100)
    rospy.Subscriber("slam_out_pose", PoseStamped, cb_pose,queue_size=100)
    rospy.Subscriber("/parametros", Float32MultiArray, cb_parametros, queue_size=100)

    #servicios
    get_plan = rospy.ServiceProxy("/planner/move_base/make_plan", GetPlan)
    clear_costmap = rospy.ServiceProxy("/planner/move_base/clear_costmaps",Empty_msg_srv)

    #reconfiguracion
    set_vel = dynamic_reconfigure.client.Client("/planner/move_base/DWAPlannerROS")
    planner = dynamic_reconfigure.client.Client("/planner/move_base/DWAPlannerROS")
    plannerglobal = dynamic_reconfigure.client.Client("/planner/move_base/GlobalPlanner")

    #acciones
    unparking_client = actionlib.SimpleActionClient('/Parking_server_node/unparking_action', unparkingAction)
    parking_client = actionlib.SimpleActionClient('/Parking_server_node/parking_action', parkingAction)
    move_base_client = actionlib.SimpleActionClient('/planner/move_base', MoveBaseAction)

    #rospy.wait_for_service("/planner/move_base/clear_costmaps")
    print 'limpiar mapas iniciado'
    unparking_client.wait_for_server()
    print 'unparking iniciado'
    parking_client.wait_for_server()
    print 'parking iniciado'
    move_base_client.wait_for_server()
    print 'move base iniciado'

    while not rospy.is_shutdown():
        if current_core_fase == core_fase.CARGAR_RUTA.value:
            get_param()
            print 'buscando mision'
            if hay_mision_cargada:
                current_core_fase = core_fase.NEXT_MAQUINA.value
                past_core_fase = core_fase.CARGAR_RUTA.value
                print 'mision cargada'
                maquinas_periodo_curent = maquinas.periodo

        elif current_core_fase == core_fase.STANDBY.value:

            current_core_fase = core_fase.NEXT_MAQUINA.value
            past_core_fase = core_fase.STANDBY.value
            maquinas_periodo_curent = maquinas.periodo               

        elif current_core_fase == core_fase.GO_MAQUINA.value:
            set_estado_msg.data = [1,1]
            set_estado_pub.publish(set_estado_msg)
            #establecer velocidad
            set_vel.update_configuration({"max_vel_trans": velocidad_a_maquina})

            #habilita reversa por un tiempo 
            tiempo_inicial_en_reversa = time.time()
            set_vel.update_configuration({"min_vel_x": escapevel})
            reversa_habilitada = True
            
            goal = MoveBaseGoal()
            goal.target_pose.pose.position.x = maquinas.x[next_maquina]
            goal.target_pose.pose.position.y = maquinas.y[next_maquina]
            goal.target_pose.pose.position.z = 0.0
            goal.target_pose.pose.orientation = Quaternion(0,0,maquinas.qz[next_maquina],maquinas.qw[next_maquina])
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()

            print 'enviando a maquina: %s' % goal
            move_base_client.send_goal(goal)
            current_core_fase = core_fase.TRAVELING_TO_MAQUINA.value
            past_core_fase = core_fase.GO_MAQUINA.value
      
        elif current_core_fase == core_fase.TRAVELING_TO_MAQUINA.value:
            status_viaje = move_base_client.get_state()
            print 'estado del viaje: %d' % status_viaje
            if status_viaje == GoalStatus.SUCCEEDED:
                current_core_fase = core_fase.DO_ACCION_MAQUINA.value
                past_core_fase = core_fase.TRAVELING_TO_MAQUINA.value
                tiempo_inicio_boton = time.time()
                print 'goal exitoso'
            elif status_viaje == GoalStatus.ABORTED:
                #habilita reversa por un tiempo 
                #tiempo_inicial_en_reversa = time.time()
                #set_vel.update_configuration({"min_vel_x": -0.15})
                #reversa_habilitada = True
                print 'reintentando'
                current_core_fase = core_fase.GO_MAQUINA.value
                clear_costmap(EmptyRequest())
                      
        elif current_core_fase == core_fase.DO_ACCION_MAQUINA.value:
            
            set_estado_msg.data = [1,4]
            set_estado_pub.publish(set_estado_msg)

            if maquinas.accion_id [next_maquina] == 0: #parking

                if current_recoger_botella_fase == recoger_botella.SEND_PARKING.value:
                    parking_goal = parkingGoal()
                    parking_goal.marker = int (maquinas.tag_id[next_maquina])
                    parking_goal.dist_from_tag = maquinas.tagPosX[next_maquina]
                    parking_goal.des_ang = maquinas.tagAng[next_maquina]
                    parking_goal.tagPosY = maquinas.tagPosY[next_maquina]
                    print 'estacionando en maquina: %s' % parking_goal
                    parking_client.send_goal(parking_goal)
                    current_recoger_botella_fase = recoger_botella.ESPERA_PARKING.value

                elif  current_recoger_botella_fase == recoger_botella.ESPERA_PARKING.value:
                    status_recoger = parking_client.get_state()
                    if status_recoger == GoalStatus.SUCCEEDED:
                        current_recoger_botella_fase = recoger_botella.ESPERA_BOTON.value
                        tiempo_inicio_boton = time.time()

                        plc_msg.data = [1,int(maquinas.accion_boton_id[next_maquina])]
                        plc_senal_pub.publish(plc_msg)
                        set_estado_msg.data = [1,4]
                        set_estado_pub.publish(set_estado_msg)

                    elif status_recoger != GoalStatus.ACTIVE:
                        current_recoger_botella_fase = recoger_botella.SEND_PARKING.value

                elif  current_recoger_botella_fase == recoger_botella.ESPERA_BOTON.value:
                    tiempo_transcurrido_boton = time.time() - tiempo_inicio_boton
                    pub_cmd_vel.publish(twist)
                    plc_msg.data = [1,int(maquinas.accion_boton_id[next_maquina])]
                    plc_senal_pub.publish(plc_msg)
                    if boton_pre_id == int(maquinas.accion_boton_id[next_maquina]) and boton_pre_estado == 1:
                        set_estado_msg.data = [1,2]
                        dejando_maquina_tiempo_limite = False
                        set_estado_pub.publish(set_estado_msg)
                        current_recoger_botella_fase = recoger_botella.SEND_UNPARKING.value

                    if maquinas.tiempoAlerta[next_maquina] > 0:
                        if tiempo_transcurrido_boton > maquinas.tiempoAlerta[next_maquina]:
                            set_estado_msg.data = [1,7]
                            set_estado_pub.publish(set_estado_msg)
                            if tiempo_transcurrido_boton > maquinas.tiempoEspera[next_maquina] + maquinas.tiempoAlerta[next_maquina]:#maquinas.tiempoEspera[next_maquina] + maquinas.tiempoAlerta[next_maquina]:
                                dejando_maquina_tiempo_limite = True
                                current_recoger_botella_fase = recoger_botella.SEND_UNPARKING.value                    

                elif  current_recoger_botella_fase == recoger_botella.SEND_UNPARKING.value:
                    unparking_goal = unparkingGoal()
                    unparking_goal.marker = int (maquinas.tag_id[next_maquina])
                    unparking_goal.dist_from_tag = distancia_desestacionado
                    print 'desestacionando de maquina: %s' % unparking_goal
                    unparking_client.send_goal(unparking_goal)
                    set_estado_msg.data = [1,3]
                    set_estado_pub.publish(set_estado_msg)
                    current_recoger_botella_fase = recoger_botella.ESPERA_UNPARKING.value

                elif  current_recoger_botella_fase == recoger_botella.ESPERA_UNPARKING.value:
                    status_unparking = unparking_client.get_state()
                    plc_msg.data = [0,int(maquinas.accion_boton_id[next_maquina])]
                    plc_senal_pub.publish(plc_msg)
                    if status_unparking == GoalStatus.SUCCEEDED:
                        if dejando_maquina_tiempo_limite:
                            current_recoger_botella_fase = recoger_botella.SEND_PARKING.value
                            current_core_fase = core_fase.NEXT_MAQUINA.value
                            past_core_fase = core_fase.DO_ACCION_MAQUINA.value
                            set_estado_msg.data = [1,1]
                            set_estado_pub.publish(set_estado_msg)
                            print "me voy por tiempo limite"
                        else:
                            current_recoger_botella_fase = recoger_botella.SEND_PARKING.value
                            current_core_fase = core_fase.GO_TEMPLADOR.value
                            past_core_fase = core_fase.DO_ACCION_MAQUINA.value
                            set_estado_msg.data = [1,2]
                            set_estado_pub.publish(set_estado_msg)
                            print "mevoy a templador, a entregar"

                    elif status_unparking != GoalStatus.ACTIVE:
                        current_recoger_botella_fase = recoger_botella.SEND_UNPARKING.value
                        print "fallo reintentando"

            elif maquinas.accion_id [next_maquina]== 1: #operador boton
                tiempo_transcurrido_boton = time.time() - tiempo_inicio_boton
                pub_cmd_vel.publish(twist)
                if boton_pre_id == int(maquinas.accion_boton_id[next_maquina]) and boton_pre_estado == 1:
                    current_core_fase = core_fase.GO_TEMPLADOR.value
                    past_core_fase = core_fase.DO_ACCION_MAQUINA.value
                    dejando_maquina_tiempo_limite = False
                    set_estado_msg.data = [1,2]
                    set_estado_pub.publish(set_estado_msg)
                if maquinas.tiempoAlerta[next_maquina] > 0:
                    if tiempo_transcurrido_boton > maquinas.tiempoAlerta[next_maquina]:
                            set_estado_msg.data = [1,7]
                            print "ya casi me voy sin botella"
                            set_estado_pub.publish(set_estado_msg)
                            if tiempo_transcurrido_boton > maquinas.tiempoEspera[next_maquina] + maquinas.tiempoAlerta[next_maquina]:
                                print "me voy sin botella"
                                dejando_maquina_tiempo_limite = True
                                current_core_fase = core_fase.NEXT_MAQUINA.value
                                set_estado_msg.data = [1,1]
                                set_estado_pub.publish(set_estado_msg)

        elif current_core_fase == core_fase.GO_TEMPLADOR.value:

            #establecer velocidad
            vel_traveling_templador = maquinas.vel_botella[next_maquina]
            if vel_traveling_templador > max_vel_nav:
                vel_traveling_templador = max_vel_nav

            set_vel.update_configuration({"max_vel_trans": vel_traveling_templador })

            #habilita reversa por un tiempo 
            tiempo_inicial_en_reversa = time.time()
            set_vel.update_configuration({"min_vel_x": escapevel})
            reversa_habilitada = True

            goal = MoveBaseGoal()
            goal.target_pose.pose.position.x = templadores.x[next_templador]
            goal.target_pose.pose.position.y = templadores.y[next_templador]
            goal.target_pose.pose.position.z = 0.0
            goal.target_pose.pose.orientation = Quaternion(0,0,templadores.qz[next_templador],templadores.qw[next_templador])
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            set_estado_msg.data = [1,2]
            set_estado_pub.publish(set_estado_msg)
            print 'enviando a templador: %s' % goal
            move_base_client.send_goal(goal)
            current_core_fase = core_fase.TRAVELING_TO_TEMPLADOR.value
            past_core_fase = core_fase.GO_TEMPLADOR.value
        
        elif current_core_fase == core_fase.TRAVELING_TO_TEMPLADOR.value:
            status_viaje = move_base_client.get_state()
            if status_viaje == GoalStatus.SUCCEEDED:
                current_core_fase = core_fase.ENTREGA.value
                past_core_fase = core_fase.TRAVELING_TO_TEMPLADOR.value
            elif status_viaje == GoalStatus.ABORTED:
                #habilita reversa por un tiempo 
                #tiempo_inicial_en_reversa = time.time()
                #set_vel.update_configuration({"min_vel_x": -0.15})
                #reversa_habilitada = True
                current_core_fase = core_fase.GO_TEMPLADOR.value
                clear_costmap(EmptyRequest())

        elif current_core_fase == core_fase.ENTREGA.value:

            if current_entrega_botella_fase == entrega_botella.SEND_PARKING.value:
                parking_goal = parkingGoal()
                parking_goal.marker = int(templadores.tag_id[next_templador])
                parking_goal.tagPosY = templadores.tagPosY[next_templador]
                parking_goal.dist_from_tag = templadores.tagPosX[next_templador]
                parking_goal.des_ang = templadores.tagAng[next_templador]
                print 'estacionando en templador: %s' % parking_goal
                parking_client.send_goal(parking_goal)
                current_entrega_botella_fase = entrega_botella.ESPERA_PARKING.value

            elif  current_entrega_botella_fase == entrega_botella.ESPERA_PARKING.value:
                status_recoger = parking_client.get_state()
                print status_recoger
                if status_recoger == GoalStatus.SUCCEEDED:
                    mover_actuadores(0,1,maquinas.vel_stacker[next_maquina]) #saca stacker
                    current_entrega_botella_fase = entrega_botella.ESPERA_STACKER.value
                    print 'exito estadionado'
                elif status_recoger != GoalStatus.ACTIVE:
                    print 'reintentando estadionado'
                    current_entrega_botella_fase = entrega_botella.SEND_PARKING.value

            elif  current_entrega_botella_fase == entrega_botella.ESPERA_STACKER.value:
                pub_cmd_vel.publish(twist)
                if pos_stacker==1:
                    current_entrega_botella_fase = entrega_botella.SEND_UNPARKING.value
                    mover_actuadores(0,0,vel_meter_stacker) #mete stacker al 40%
                    time.sleep(2)

            elif  current_entrega_botella_fase == entrega_botella.SEND_UNPARKING.value:
               
                set_estado_msg.data = [1,3]
                set_estado_pub.publish(set_estado_msg)
                unparking_goal = unparkingGoal()
                unparking_goal.marker = int(templadores.tag_id[next_templador])
                unparking_goal.dist_from_tag = distancia_desestacionado
                print 'desestacionando de templador: %s' % unparking_goal
                unparking_client.send_goal(unparking_goal)
                current_entrega_botella_fase = entrega_botella.ESPERA_UNPARKING.value

            elif  current_entrega_botella_fase == entrega_botella.ESPERA_UNPARKING.value:
                status_unparking = unparking_client.get_state()
                if status_unparking == GoalStatus.SUCCEEDED:
                    current_core_fase = core_fase.NEXT_MAQUINA.value
                    current_entrega_botella_fase = entrega_botella.SEND_PARKING.value
                    past_core_fase = core_fase.ENTREGA.value
                    set_estado_msg.data = [1,1]
                    set_estado_pub.publish(set_estado_msg)

                elif status_unparking != GoalStatus.ACTIVE:
                    current_entrega_botella_fase = entrega_botella.SEND_UNPARKING.value

        elif current_core_fase == core_fase.NEXT_MAQUINA.value: 
            
            if numero_maquina > 0:
                next_maquina = next_maquina + 1    
                print 'haymas de 1 maquina'      
                if next_maquina > numero_maquina:
                    next_maquina = 0
                    print 'de regreso a la maquina '
            else :
                next_maquina = 0
                print 'solo hay 1 maquina'
            
            try:
                next_templador = templadores.ID.index(int(maquinas.templador_id[next_maquina]))
                print('nexttemplador %d' % (next_templador))
                #next_templador = int(maquinas.templador_id[next_maquina])
            except:
                print('el id %d templador no existe' % (int(maquinas.templador_id[next_maquina])))
                next_templador = 1

            current_core_fase = core_fase.GO_MAQUINA.value
            past_core_fase = core_fase.NEXT_MAQUINA.value
            print 'siguiente maquina'
            
            maquina_ant = next_maquina

            '''
            if goal_d:
                next_maquina = 0
                next_templador = int(maquinas.templador_id[next_maquina])
                goal_d = False
            else:
                goal_d = True
                next_maquina = 1
                next_templador = int(maquinas.templador_id[next_maquina])
           
            
            goal_i=0
            buscando_next_maquina = True
            maquinas.periodo =          rospy.get_param('mision/maquinas/periodo', 0)

            goal_i_siguiente=1
            print "buscando siguiente goal"
            trayecto_al_templador = 10 + calcular_tiempo_trayecto(maquinas.vel_botella[goal_i], maquinas.x[goal_i], maquinas.y[goal_i], templadores.x[int(maquinas.templador_id[goal_i])], templadores.y[int(maquinas.templador_id[goal_i])])
            trayecto_al_maquina = calcular_tiempo_trayecto(vel_sin_botella, pos_x, pos_y, maquinas.x[goal_i], maquinas.y[goal_i])
            tiempo_acumulado_trayecto = trayecto_al_templador + trayecto_al_maquina
            print maquinas_periodo_curent
            print tiempo_acumulado_trayecto
            if trayecto_al_maquina >= maquinas_periodo_curent[goal_i] + margen_de_arribo:
                next_maquina = goal_i
                next_templador = int(maquinas.templador_id[next_maquina])
                print 'siguiente maquina es escogida en primera etapa: '
                print goal_i
                tiempo_acumulado_trayecto = 0
                maquinas_periodo_curent[goal_i] = maquinas.periodo[goal_i]
                print maquinas.periodo[goal_i]
                
            else:
                while buscando_next_maquina:
                    trayecto_siguiente_maquina = calcular_tiempo_trayecto(vel_sin_botella,templadores.x[int(maquinas.templador_id[goal_i])], templadores.y[int(maquinas.templador_id[goal_i])], maquinas.x[goal_i_siguiente], maquinas.y[goal_i_siguiente])
                    trayecto_al_siguiente_templador = 5 + calcular_tiempo_trayecto(maquinas.vel_botella[goal_i_siguiente], maquinas.x[goal_i_siguiente], maquinas.y[goal_i_siguiente], templadores.x[int(maquinas.templador_id[goal_i_siguiente])], templadores.y[int(maquinas.templador_id[goal_i_siguiente])])
                    tiempo_acumulado_trayecto = trayecto_al_siguiente_templador + trayecto_siguiente_maquina + tiempo_acumulado_trayecto
                    print "calculando tiempo de siguiente prioridad"
                    print tiempo_acumulado_trayecto
                    print maquinas_periodo_curent[goal_i]

                    if tiempo_acumulado_trayecto > maquinas_periodo_curent[goal_i]:
                        next_maquina = goal_i 
                        next_templador = int(maquinas.templador_id[next_maquina]) 
                        maquinas_periodo_curent[goal_i] = maquinas.periodo[goal_i]
                        buscando_next_maquina = False
                        tiempo_acumulado_trayecto=0
                        print "siguiente maquina en while"
                        print next_maquina
                    else:
                        goal_i_siguiente = goal_i_siguiente + 1
                        goal_i = goal_i + 1

                        if goal_i_siguiente > numero_maquina:
                            next_maquina = goal_i
                            next_templador = int(maquinas.templador_id[next_maquina])
                            maquinas_periodo_curent[goal_i] = maquinas.periodo[goal_i]
                            buscando_next_maquina = False
                            tiempo_acumulado_trayecto = 0
                            print "maquina seleccionada por maximo iteracion"
                            print next_maquina
                '''

        activar_reversa()
        conteo_tiempo()
        r.sleep()
                    