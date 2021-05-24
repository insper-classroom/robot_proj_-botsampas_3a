from __future__ import print_function, division

# Para rodar, recomendamos que faça:
# 
#    roslaunch my_simulation pista_u.launch
#
# Depois o controlador do braço:
#
#    roslaunch mybot_description mybot_control2.launch 	

import time
import os
import rospy 
import numpy as np
import cv2
import cv2.aruco as aruco
import statsmodels.api as sm
import math
import visao_module

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from subprocess import call
from colorama import Fore, Back, Style
from colorama import init
from Robot import Robot
from Creeper import Creeper

# DECLARANDO VARIÁVEIS

estagio1 = True
estagio2 = False
estagio3 = False
estagio_creeper = False
estagio_estacao = False
nao_encontrou  = True
ultima_placa = 0
min_vel_linear = 0.2
max_vel_linear = 0.5

tupla = ["verde", 23, "horse"]

define_cor = tupla[0]
define_id = tupla[1]
define_estacao = tupla[2]


bridge = CvBridge()

# DECLARANDO FUNÇÕES

def main():
    global robot
    global creeper
    global estagio1
    global estagio2
    global estagio3
    global estagio_creeper
    global estagio_estacao
    global ultima_placa
    global nao_encontrou
    global min_vel_linear
    global max_vel_linear

    media, centro, maior_area = creeper.identify_color(robot)
    ids, _ = robot.getAruco()
    M = robot.get_yellow_moments()
    robot.publish_joints()
    resultado_mobile = robot.detectMobileNet()

    # Verificando cor e ID dos creepers
    try:
        for i in ids:
            if i == int(define_id) and maior_area > 1500 and nao_encontrou:
                estagio1 = False
                estagio2 = False
                estagio3 = False
                estagio_creeper = True
    except Exception:
        pass
    
    try:
        for i in resultado_mobile:
            if i[0] == define_estacao and not nao_encontrou:
                estagio1 = False
                estagio2 = False
                estagio3 = False
                estagio_creeper = False
                estagio_estacao = True
    except Exception:
        pass
    
    # ESTÁGIOS (1,2,3)
    if estagio1:

        print("Estágio 1")

        try:
            for i in ids:
                if i == 150:
                    ultima_placa = "ESQUERDA"
                elif i == 50:
                    ultima_placa = "DIREITA"
        except Exception:
            pass

        robot.follow_yellow_line(min_vel_linear=min_vel_linear, max_vel_linear=max_vel_linear)
        
        if M['m00'] <= 0:
            estagio1, estagio2 = False, True
    
    elif estagio2:

        print ("Estágio 2")

        robot.rotate(clockwise=True)

        if M['m00'] > 0:
            estagio2, estagio3 = False, True
    
    elif estagio3:

        print("Estágio 3")

        img = robot.getImage()
        w = robot.getWidth()
        
        if ultima_placa == "ESQUERDA":
            img[:, 3*w//4:] = [0, 0, 0]
        else:
            img[:, :w//4] = [0, 0, 0]
        
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        robot.follow_yellow_line(img=img, min_vel_linear=min_vel_linear, max_vel_linear=max_vel_linear)

        if M['m00'] <= 0:
            estagio1, estagio2 = False, True

    elif estagio_creeper:

        print ("Estágio Creeper")
        bateu = robot.go_to_creeper(creeper, min_distance=0.19)

        if bateu:
            estagio_creeper, estagio2 = False, True
            nao_encontrou = False
            min_vel_linear = 0.1
            max_vel_linear = 0.1
    
    elif estagio_estacao:
        print ("Estágio Estacao")
        bateu = robot.go_to_station(define_estacao, resultado_mobile)

        if bateu:
            estagio_estacao = False

    cv2.imshow("Main", robot.getImage(original=True))


if __name__=="__main__":

    rospy.init_node("ConceitoB")

    robot = Robot()
    creeper = Creeper(define_cor, define_id)

    rospy.sleep(0.1)        
    
    try:
        while not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        print("  Já entendi... Já pode parar de dar CTRL+C xuxu")