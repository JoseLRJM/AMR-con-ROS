#!/usr/bin/env python  

import rospy
import numpy as np
import tf
from enum import Enum
from std_msgs.msg import UInt32MultiArray, Int32
from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import time
import actionlib
from champy_action_msgs.msg import parkingAction, parkingFeedback, parkingResult
from champy_action_msgs.msg import unparkingAction, unparkingFeedback, unparkingResult


#GANANCIAS DE CONTROL
KP_X_ACERCANDOSE = -0.4
KP_TH_ACERCANDOSE = -0.6
KP_X_CENTRANDO = -0.8
KP_TH_CENTRANDO = 0.6

#parametros p
TIME_OUT_TO_SEARCH = 20
TIME_OUT_TO_SEARCH_UNPARKING = 20
TIME_OUT_MAX_STOP_ACERCANDOSE = 1
TIME_OUT_MAX_STOP_CENTRANDO = 1

MAX_DIFF_TAG = 0.5
MAX_TH_ERROR = 0.09

MAX_VEL_X_CENTRANDO= 0.15
MAX_VEL_TH_CENTRANDO= 0.25

MAX_VEL_X_ACERCANDOSE= 0.1
MAX_VEL_TH_ACERCANDOSE= 0.2

margen_x_arribo = 0.001

class AutomaticParkingVision_server():

    def __init__(self):
        self.MARKER_ID_DETECTION = 0
        self.success = False
        self.ini_var()
        self.odom_test = Odometry()

        self.sub_odom_robot = rospy.Subscriber('/zed/odom', Odometry, self.cb_GetRobotOdom, queue_size = 1)
        self.sub_info_marker = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.cb_GetMarkerOdom, queue_size = 1)
        self.pub_cmd_vel = rospy.Publisher('/planner/cmd_vel', Twist, queue_size=1)
        self.odom_pub = rospy.Publisher("odom_parking", Odometry, queue_size=50)

        self.parking_server = actionlib.SimpleActionServer('~parking_action', parkingAction, execute_cb=self.cb_parking_action, auto_start = False)
        self.parking_server.start()
        self.unparking_server = actionlib.SimpleActionServer('~unparking_action', unparkingAction, execute_cb=self.cb_unparking_action, auto_start = False)
        self.unparking_server.start()

        self.ParkingSequence = Enum('ParkingSequence', 'INICIALIZANDO CENTRANDO ACERCANDOSE ORIENTANDOSE_FIN ESPERA_SENAL UNPARKING FIN')
        self.current_parking_sequence = self.ParkingSequence.INICIALIZANDO.value
        self.loop_rate = rospy.Rate(10)
        
    def cb_GetMarkerOdom(self, markers_odom_msg):
        for marker_odom_msg in markers_odom_msg.markers:
            if marker_odom_msg.id == self.MARKER_ID_DETECTION:
                x_alpha = 0.8
                y_alpha = 0.8
                th_alpha = 0.8
                
                quaternion = (marker_odom_msg.pose.pose.orientation.x, marker_odom_msg.pose.pose.orientation.y, marker_odom_msg.pose.pose.orientation.z, marker_odom_msg.pose.pose.orientation.w)
                theta = tf.transformations.euler_from_quaternion(quaternion)[2]

                self.x_fil = x_alpha * marker_odom_msg.pose.pose.position.x + (1 - x_alpha) * self.x_fil
                self.y_fil = y_alpha * marker_odom_msg.pose.pose.position.y + (1 - y_alpha) * self.y_fil
                self.th_fil = th_alpha * (theta + np.pi / 2) + (1 - th_alpha) * self.th_fil
                if abs(self.x_fil) < 5 and abs(self.y_fil) < 3:
                    self.mark_pos_theta = self.th_fil
                    self.mark_pos_x = self.x_fil
                    self.mark_pos_y = self.y_fil

                    #print('mar_th %f  mar_x %f  mar_y %f' % (self.mark_pos_theta,self.mark_pos_x, self.mark_pos_y))
                    if self.is_marker_pose_received == False:
                        self.is_marker_pose_received = True
                    
                    self.lasttime_tag = rospy.get_time()

                    if self.inicializado: 

                        self.ofset_odom_x = self.odom_pos_x
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
            q_th_odom = quaternion_from_euler(0,0,self.th)
            self.odom_test.header.stamp = rospy.Time.now()
            self.odom_test.header.frame_id = "odom"
            self.odom_test.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*q_th_odom))
            self.odom_test.child_frame_id = "base_footprint"
            self.odom_test.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            self.odom_pub.publish(self.odom_test)

    def cb_parking_action(self,action_msg):

        self.current_parking_sequence = self.ParkingSequence.INICIALIZANDO.value
        feedback = parkingFeedback()
        result = parkingResult()
        MIN_THETA = 0
        self.ini_var()
        self.MARKER_ID_DETECTION = action_msg.marker
        y_deseada_tag = action_msg.tagPosY
        dist_from_tag_ = action_msg.dist_from_tag + margen_x_arribo
        des_ang = action_msg.des_ang
        print 'adist_from_tag_ %f' % dist_from_tag_
        dis_from_point = 0.4
        dist_deseada_tag = dist_from_tag_ + dis_from_point
        
        while not self.current_parking_sequence == self.ParkingSequence.FIN.value and not rospy.is_shutdown():

            if self.current_parking_sequence == self.ParkingSequence.INICIALIZANDO.value:
                if not self.is_marker_pose_received:
                    self.time_out =  self.time_out + 1
                    if  self.time_out < TIME_OUT_TO_SEARCH:
                        print "ACERCANDOSE SIN VER"
                        print self.time_out
                        self.set_vel(0.3,0)
                    else :
                        print "no hay tag, cancelado"
                        break
                else:
                    print "tag encontrado, inicializado"
                    self.current_parking_sequence = self.ParkingSequence.CENTRANDO.value
                    feedback.fase = self.current_parking_sequence
                    self.parking_server.publish_feedback(feedback)

                    self.inicializado=True
                    self.set_vel(0,0)
                    print "centrando tag"
            
            elif self.current_parking_sequence == self.ParkingSequence.CENTRANDO.value:
                if abs(self.x) <= dist_deseada_tag: # or abs(y_deseada_tag-self.y) < 0.02:
                    print "checando"
                    if self.check_tag():
                        if abs(self.x) <= dist_deseada_tag:
                            self.current_parking_sequence = self.ParkingSequence.ACERCANDOSE.value
                            feedback.fase = self.current_parking_sequence
                            self.parking_server.publish_feedback(feedback)
                            print 'centrado terminado tagv %f' % self.x
                            #print "centrado terminado tag th:%f x:%f y:%f", self.th,self.x,self.y)
                else:
                    c_x = (self.x - dist_deseada_tag) * KP_X_CENTRANDO
                    #c_th = (self.mark_pos_y )* KP_TH_CENTRANDO
                    angulo_deseado = -((np.pi/2) + math.atan2(self.x + dist_deseada_tag, self.y - y_deseada_tag))
                    if abs(self.x + dist_deseada_tag) < 0.2:
                        if angulo_deseado < -0.4:
                            angulo_deseado = -0.4
                        if angulo_deseado > 0.4:
                            angulo_deseado = 0.4

                    c_th = (angulo_deseado - self.th) * KP_TH_CENTRANDO
                    if c_th < -MAX_VEL_TH_CENTRANDO:
                        c_th = -MAX_VEL_TH_CENTRANDO
                    if c_th > MAX_VEL_TH_CENTRANDO:
                        c_th = MAX_VEL_TH_CENTRANDO

                    if c_x < -MAX_VEL_X_CENTRANDO:
                        c_x = -MAX_VEL_X_CENTRANDO
                    if c_x > MAX_VEL_X_CENTRANDO:
                        c_x = MAX_VEL_X_CENTRANDO

                    #print angulo_deseado
                    print('angulo_deseado %f  x %f  y %f  dist_deseada_tag %f  self.th %f' % (angulo_deseado,self.x,self.y,dist_deseada_tag,self.th))
                    #print('th %f  angulo_deseado %f  c_th %f ' % (self.th,angulo_deseado,c_th))
                    #print('y %f  y_deseada %f  error %f ' % (self.y,y_deseada_tag,(y_deseada_tag-self.y)))
                    self.diff_tag_time = rospy.get_time() - self.lasttime_tag
                    if self.diff_tag_time < TIME_OUT_MAX_STOP_CENTRANDO:
                        self.set_vel(c_x,c_th)
                    else :
                        self.set_vel(0,0) 
                        print "error centrando por mucho tiempo"

            elif self.current_parking_sequence == self.ParkingSequence.ACERCANDOSE.value:
                if abs(self.x) <= dist_from_tag_:
                    self.set_vel(0,0)
                    print "checando arrivo correcto en cercandose"
                    if self.check_tag():
                        if abs(self.x) <= dist_from_tag_:
                            self.current_parking_sequence = self.ParkingSequence.ORIENTANDOSE_FIN.value
                            feedback.fase = self.current_parking_sequence
                            self.parking_server.publish_feedback(feedback)
                           # print 'acercandose terminado tag %f' % self.x
                            #print "acercandose terminado tag th:%f x:%f y:%f", self.th,self.x,self.y)
                else:
                    c_x = (self.x - dist_from_tag_) * KP_X_ACERCANDOSE
                    angulo_deseado_acercandose = -((np.pi/2) + math.atan2(self.x + dist_from_tag_, self.y - y_deseada_tag))
                    c_th = (-des_ang + self.th) * KP_TH_ACERCANDOSE
                    #c_th = -(angulo_deseado_acercandose - self.th) * KP_TH_ACERCANDOSE
                    #print 'acercandose x %f' % c_x
                    #print 'th %f' % c_th 

                    if c_th < -MAX_VEL_TH_ACERCANDOSE:
                        c_th = -MAX_VEL_TH_ACERCANDOSE
                    if c_th > MAX_VEL_TH_ACERCANDOSE:
                        c_th = MAX_VEL_TH_ACERCANDOSE

                    if c_x < -MAX_VEL_X_ACERCANDOSE:
                        c_x = -MAX_VEL_X_ACERCANDOSE
                    if c_x > MAX_VEL_X_ACERCANDOSE:
                        c_x = MAX_VEL_X_ACERCANDOSE

                    self.diff_tag_time = rospy.get_time() - self.lasttime_tag

                    if self.diff_tag_time < TIME_OUT_MAX_STOP_ACERCANDOSE:
                        self.set_vel(c_x,c_th)
                        #print 'c_x %f' % c_x
                        #print 'c_th %f' % c_th
                    else :
                        self.set_vel(0,0)

                    if abs(self.x) > dist_deseada_tag:
                        print 'regresando a centrado'
                        self.current_parking_sequence = self.ParkingSequence.CENTRANDO.value
                        
            elif self.current_parking_sequence == self.ParkingSequence.ORIENTANDOSE_FIN.value:
                if abs(-des_ang + self.th) <= MAX_TH_ERROR:
                    self.set_vel(0,0)
                    print "intento de oriencion final"
                    #if self.check_tag():
                    #    if abs(-des_ang + self.th) <= MAX_TH_ERROR:
                    self.current_parking_sequence = self.ParkingSequence.FIN.value
                    feedback.fase = self.current_parking_sequence
                    self.parking_server.publish_feedback(feedback)
                    self.success = True
                    self.inicializado = False
                    print 'orientandose terminado tag %f' % self.th
                            #print "orientandose terminado tag th:%f x:%f y:%f", self.th,self.x,self.y)     
                else:
                    c_th = -(-des_ang + self.th) * KP_TH_CENTRANDO 
                    
                    if c_th < -MAX_VEL_TH_CENTRANDO:
                        c_th = -MAX_VEL_TH_CENTRANDO
                    if c_th > MAX_VEL_TH_CENTRANDO:
                        c_th = MAX_VEL_TH_CENTRANDO

                    if c_th > 0:
                        c_th = c_th + MIN_THETA
                    if c_th < 0:
                        c_th = c_th-MIN_THETA

                    #print 'orientacion fin th %f' % c_th
                    self.diff_tag_time = rospy.get_time() - self.lasttime_tag
                    if self.diff_tag_time < TIME_OUT_MAX_STOP_CENTRANDO:
                        self.set_vel(0.05,c_th)
                    else :
                        self.set_vel(0,0)

                    if abs(self.x) > dist_from_tag_:
                        print 'regresando a acercandose'
                        self.current_parking_sequence = self.ParkingSequence.ACERCANDOSE.value

            if self.parking_server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self.parking_server)
                self.parking_server.set_preempted()
                self.success = False
                break

            self.loop_rate.sleep()

        if self.success:
            result.th_fin = self.th
            result.x_fin = self.x
            print "exitoso"
            #rospy.loginfo('%s: Succeeded' % self.parking_server)
            self.parking_server.set_succeeded(result)
            print "publicado el resultado"
        else:
            result.th_fin = self.th
            result.x_fin = self.x
            print "abortado"
            #rospy.loginfo('%s: aborted' % self.parking_server)
            self.parking_server.set_aborted(result)
            #self.parking_server.set_succeeded(result)


        self.current_parking_sequence = self.ParkingSequence.INICIALIZANDO.value
        feedback = parkingFeedback()
        result = parkingResult()
        MIN_THETA = 0
        self.ini_var()
        self.MARKER_ID_DETECTION = action_msg.marker
        y_deseada_tag = action_msg.tagPosY
        dist_from_tag_ = action_msg.dist_from_tag + margen_x_arribo
        des_ang = action_msg.des_ang
        print 'adist_from_tag_ %f' % dist_from_tag_
        dis_from_point = 0.5
        dist_deseada_tag = dist_from_tag_ + dis_from_point
 
    def cb_unparking_action(self,action_msg):
        feedback = unparkingFeedback()
        result = unparkingResult()
        self.current_parking_sequence = self.ParkingSequence.INICIALIZANDO.value
        self.ini_var()
        self.MARKER_ID_DETECTION = action_msg.marker
        dist_deseada_tag = action_msg.dist_from_tag
        
        while not (self.current_parking_sequence == self.ParkingSequence.FIN.value) and not rospy.is_shutdown():


            if self.current_parking_sequence == self.ParkingSequence.INICIALIZANDO.value:
                if not self.is_marker_pose_received:
                    self.time_out =  self.time_out + 1
                    if  self.time_out < TIME_OUT_TO_SEARCH_UNPARKING:
                        print "buscando tag, retrocediendo"
                        self.set_vel(-0.2,0)
                    else :
                        self.set_vel(0,0)
                        print "no hay tag, error, paro de mergencia"
                        self.success = False
                        self.current_parking_sequence = self.ParkingSequence.FIN.value
                        break
                else:
                    if self.check_tag(): 
                        print "tag encontrado, inicializado"
                        self.current_parking_sequence = self.ParkingSequence.UNPARKING.value
                        self.inicializado=True
                        self.tiempo_inicio = rospy.Time.now().to_sec()
                        print 'tiempo_inicio  %f' % self.tiempo_inicio
                        print "centrando tag"

            elif self.current_parking_sequence == self.ParkingSequence.UNPARKING.value:
                if abs(self.x) >= dist_deseada_tag:
                    self.set_vel(0,0)
                    print "intento dee termino"
                    print 'x %f' % self.x 
                    if self.check_tag():
                        if abs(self.x) >= dist_deseada_tag:
                            if (rospy.Time.now().to_sec()-self.tiempo_inicio) > 2.5:
                                self.current_parking_sequence = self.ParkingSequence.FIN.value
                                self.success = True
                                print 'x %f' % self.x 
                                print 'unparking terminado'
                            else:
                                self.set_vel(-0.4,0)


                else:
                    c_x = ((dist_deseada_tag/(self.x + dist_deseada_tag))*-0.08) - 0.2
                    print 'c_x %f' % c_x
                    c_th = - self.th * KP_TH_CENTRANDO 
                    if c_th < -MAX_VEL_TH_CENTRANDO:
                        c_th = -MAX_VEL_TH_CENTRANDO
                    if c_th > MAX_VEL_TH_CENTRANDO:
                        c_th = MAX_VEL_TH_CENTRANDO

                    if c_x < -MAX_VEL_X_CENTRANDO-0.2:
                        c_x = -MAX_VEL_X_CENTRANDO- 0.2
                    if c_x > 0.01:
                        c_x = 0.0
                    

                    self.set_vel(c_x,c_th)

            if self.unparking_server.is_preempt_requested():
                self.unparking_server.set_preempted()
                self.success = False
                print "solicitud de cancelacion"
                break
            
            feedback.x_current = self.x
            self.unparking_server.publish_feedback(feedback)

            self.loop_rate.sleep()

        result.th_fin = self.th
        result.x_fin = self.x
        print "terminado"
        if self.success:
            #rospy.loginfo('%s: Succeeded' % self.unparking_server)
            print "terminado exitoso publicando estado"
            self.unparking_server.set_succeeded(result)
        else:
            #rospy.loginfo('%s: aborted' % self.unparking_server)
            print "fallo publicando estado"
            self.unparking_server.set_aborted(result)

    def set_vel(self,vel_x,vel_theta):
        twist = Twist()
        twist.linear.x = vel_x
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = vel_theta
        self.pub_cmd_vel.publish(twist)

    def check_tag(self):
        check_tag_rate = rospy.Rate(20)
        self.is_marker_pose_received = False
        cont_tag=0
        list_tag=[1,2,3]

        while (not self.is_marker_pose_received) and (not rospy.is_shutdown()):
            #if self.is_marker_pose_received:
            #    list_tag[cont_tag] = self.x
            #    cont_tag = cont_tag + 1
            check_tag_rate.sleep()

        #diff0 = abs(list_tag[0]-list_tag[1])
        #diff1 = abs(list_tag[1]-list_tag[2])
        #diff2 = abs(list_tag[0]-list_tag[2])
        #if diff0 < MAX_DIFF_TAG and diff1 < MAX_DIFF_TAG and diff2 < MAX_DIFF_TAG:
        return True
        #else:
        #    return False

    def ini_var(self):
        self.success = False
        #inicializacion de variables chequeo
        self.lasttime_tag=0.0
        self.is_marker_pose_received = False
        self.is_odom_received = False 
        self.inicializado = False
        self.time_out = 0 #tiempo sin marcador al iniciar el estacionado

        #inicializacion de variables de parking
        self.ofset_odom_x = 0.0
        self.ofset_odom_y = 0.0
        self.ofset_odom_theta = 0.0

        self.x_pro = 0.0
        self.y_pro = 0.0

        self.odom_theta = 0.0
        self.odom_pos_x = 0.0
        self.odom_pos_y = 0.0

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.mark_pos_theta = 0.0
        self.mark_pos_x = 0.0
        self.mark_pos_y = 0.0

        self.x_fil = 2.0
        self.y_fil = 0.0
        self.th_fil = 0.0

if __name__ == '__main__':
    rospy.init_node("Parking_server_node")
    s = AutomaticParkingVision_server()
    rospy.spin()
