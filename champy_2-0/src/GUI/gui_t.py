#!/usr/bin/env python  
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

import time
import traceback, sys
import os
import rospy
from std_msgs.msg import Float32, Int32, Int32MultiArray
import time

from PyQt5.QtCore import QThread
from PyQt5.QtGui import QMovie

import subprocess    


class MainWindow(QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.ventana_A = QWidget()
        self.ventana_A.setStyleSheet('background-color: white;')
        self.ros_iniciado = False
        self.mapa_cargado = False
        self.mision_ejecutada =False
        self.robot_pausado = True

        self.boton_inicio = QPushButton(self.ventana_A)
        self.boton_inicio.setText("INICIAR ROS")
        self.boton_inicio.setStyleSheet('QPushButton {background-color: #b5b5b5}')
        self.boton_inicio.setGeometry(50,40,150,50)
        self.boton_inicio.clicked.connect(self.boton_inicio_clicked)

        self.boton_apagar = QPushButton(self.ventana_A)
        self.boton_apagar.setText("APAGAR ROBOT")
        self.boton_apagar.setStyleSheet('QPushButton {background-color: #b5b5b5}')
        self.boton_apagar.setGeometry(250,40,150,50)
        self.boton_apagar.clicked.connect(self.boton_apagar_clicked)

        self.boton_reiniciar = QPushButton(self.ventana_A)
        self.boton_reiniciar.setText("REINICIAR ROBOT")
        self.boton_reiniciar.setStyleSheet('QPushButton {background-color: #b5b5b5}')
        self.boton_reiniciar.setGeometry(450,40,150,50)
        self.boton_reiniciar.clicked.connect(self.boton_reiniciar_clicked)

        self.boton_selec_map = QPushButton(self.ventana_A)
        self.boton_selec_map.setText("SELECCIONAR MAPA")
        self.boton_selec_map.setStyleSheet('QPushButton {background-color: #b5b5b5}')
        self.boton_selec_map.setEnabled(False)
        self.boton_selec_map.setGeometry(650,40,150,50)
        self.boton_selec_map.clicked.connect(self.cargar_mapa)

        self.boton_selec_mision = QPushButton(self.ventana_A)
        self.boton_selec_mision.setText("SELECCIONAR MISION")
        self.boton_selec_mision.setStyleSheet('QPushButton {background-color: #b5b5b5}')
        self.boton_selec_mision.setEnabled(False)
        self.boton_selec_mision.setGeometry(50,500,165,50)
        self.boton_selec_mision.clicked.connect(self.selec_mision_clicked)

        self.boton_auto_robot = QPushButton(self.ventana_A)
        self.boton_auto_robot.setText("INICIAR MISION")
        self.boton_auto_robot.setStyleSheet('QPushButton {background-color: #b5b5b5}')
        self.boton_auto_robot.setEnabled(False)
        self.boton_auto_robot.setGeometry(250,500,150,50)
        self.boton_auto_robot.clicked.connect(self.auto_robot_clicked)

        self.boton_pausar_robot = QPushButton(self.ventana_A)
        self.boton_pausar_robot.setText("MODO AUTO")
        self.boton_pausar_robot.setStyleSheet('QPushButton {background-color: #b5b5b5}')
        self.boton_pausar_robot.setEnabled(False)
        self.boton_pausar_robot.setGeometry(450,500,150,50)
        self.boton_pausar_robot.clicked.connect(self.pausar_robot_clicked)

        self.boton_reiniciar_paro = QPushButton(self.ventana_A)
        self.boton_reiniciar_paro.setText("ABRIR RVIZ")
        self.boton_reiniciar_paro.setStyleSheet('QPushButton {background-color: #b5b5b5}')
        self.boton_reiniciar_paro.setGeometry(650,500,150,50)
        self.boton_reiniciar_paro.clicked.connect(self.reiniciar_paro_clicked)

        self.texbox_mapa_cargado = QTextEdit(self.ventana_A)
        self.texbox_mapa_cargado.setTextColor(QColor(0, 0, 0))
        self.texbox_mapa_cargado.insertPlainText("No se ha seleccionado el mapa.")
        self.texbox_mapa_cargado.setReadOnly (True)
        self.texbox_mapa_cargado.move(10,5)
        self.texbox_mapa_cargado.resize(500,30)

        self.texbox_mision_cargado = QTextEdit(self.ventana_A)
        self.texbox_mision_cargado.setTextColor(QColor(0, 0, 0))
        self.texbox_mision_cargado.insertPlainText("No se ha seleccionado mision.")
        self.texbox_mision_cargado.setReadOnly (True)
        self.texbox_mision_cargado.move(550,5)
        self.texbox_mision_cargado.resize(500,30)

        self.label = QLabel(self.ventana_A)
        self.movie = QMovie("/home/user/catkin_ws/src/champy_2-0/src/GUI/lentes.gif")
        self.label.setGeometry(100,95,780,370)
        self.label.setScaledContents(True)
        self.movie.frameChanged.connect(self.repaint)
        self.label.setMovie(self.movie)
        self.movie.start()
               
        self.threadpool = QThreadPool()
        self.ventana_A.showMaximized ()#showFullScreen()

    def paintEvent(self, event):
        currentFrame = self.movie.currentPixmap()
        frameRect = currentFrame.rect()
        frameRect.moveCenter(self.rect().center())
        if frameRect.intersects(event.rect()):
            painter = QPainter(self)
            painter.drawPixmap(frameRect.left(), frameRect.top(), currentFrame)
    
    def boton_inicio_clicked(self):
        self.ros_iniciado
        if self.ros_iniciado:
            self.boton_inicio.setText("INICIAR ROS")
            self.boton_apagar.setEnabled(True)
            self.boton_reiniciar.setEnabled(True)
            self.boton_selec_map.setEnabled(False)
            self.boton_selec_mision.setEnabled(False)
            self.boton_pausar_robot.setEnabled(False)
            self.boton_auto_robot.setEnabled(False)
            self.ros_iniciado = False
            os.system("rosnode kill -a")
        else:
            self.boton_inicio.setText("DETENER ROS")
            self.boton_apagar.setEnabled(False)
            self.boton_reiniciar.setEnabled(False)
            self.boton_selec_map.setEnabled(True)
            self.boton_pausar_robot.setEnabled(False)
            self.ros_iniciado = True
            os.system('gnome-terminal --tab -- roslaunch champy_2-0 champy_hardware.launch')
            time.sleep(10)
            self.publicador_app_pub = rospy.Publisher("/publicador_app", Int32, queue_size=100)
            self.set_estado_pub = rospy.Publisher("/set_estado", Int32MultiArray, queue_size=30)
            rospy.init_node('gui_node')

    def cargar_mapa(self):
        self.mapa_cargado
        self.publicador_app_msg = Int32()
        selec_file = QFileDialog()
        file_name, _= selec_file.getOpenFileName(self, 'Open File',"/home/user/mapas/", "all files (*.db)")

        if file_name:
            os.system("rosparam set /rtabmap/database_path \"" + file_name +"\" ")
            if self.mapa_cargado:
                self.publicador_app_msg.data=5
                self.publicador_app_pub.publish(self.publicador_app_msg)
                time.sleep(10)
                self.publicador_app_msg.data=8
                self.publicador_app_pub.publish(self.publicador_app_msg) 
                self.texbox_mapa_cargado.setText ( "MAPA SELECCIONADO:  " + file_name ) 
                self.boton_selec_mision.setEnabled(True)

            else: 
                self.mapa_cargado =True
                self.publicador_app_msg.data=8
                self.publicador_app_pub.publish( self.publicador_app_msg)
                self.texbox_mapa_cargado.setText ( "MAPA SELECCIONADO:  " + file_name ) 
                self.boton_selec_mision.setEnabled(True)

    def selec_mision_clicked(self):
        selec_file = QFileDialog()
        file_name, _= selec_file.getOpenFileName(self, 'Open File',"/home/user/misiones/", "all files (*.yaml)")
        
        if file_name:
            self.texbox_mision_cargado.setText ( "MISION SELECCIONADA:  " + file_name ) 
            os.system("rosparam load " + file_name +" mision")
            self.boton_pausar_robot.setEnabled(True) 
            self.boton_auto_robot.setEnabled(True)         

    def auto_robot_clicked(self):
        self.mision_ejecutada
        if self.mision_ejecutada:
            self.publicador_app_msg.data=7
            self.publicador_app_pub.publish(self.publicador_app_msg)
            
            self.boton_auto_robot.setText("INICIAR MISION")
            self.mision_ejecutada = False
            self.boton_pausar_robot.setEnabled(False)
        else:
            self.publicador_app_msg.data=6
            self.publicador_app_pub.publish(self.publicador_app_msg)
            self.mision_ejecutada =True
            self.boton_pausar_robot.setEnabled(True)
            self.boton_auto_robot.setText("DETENER MISION")
            
    def pausar_robot_clicked(self):
        self.set_estado_msg = Int32MultiArray()

        if self.robot_pausado:
            self.set_estado_msg.data = [0,1]
            self.set_estado_pub.publish(self.set_estado_msg)
            self.robot_pausado=False
            self.boton_pausar_robot.setText("MODO MANUAL")
        else :
            self.set_estado_msg.data = [0,0]
            self.set_estado_pub.publish(self.set_estado_msg)
            self.robot_pausado=True
            self.boton_pausar_robot.setText("MODO AUTO")
        
    def reiniciar_paro_clicked(self):
        subprocess.call('gnome-terminal --tab -- roslaunch champy_2-0 rviz.launch', shell=True)

    def boton_apagar_clicked(self):
        os.system("shutdown now")

    def boton_reiniciar_clicked(self):
        os.system("shutdown -r now")


app = QApplication([])
window = MainWindow()
app.exec_()
