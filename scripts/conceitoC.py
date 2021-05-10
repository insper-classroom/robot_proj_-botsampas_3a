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

#--------------------------------------------------------------------------

# DECLARANDO VARIÁVEIS

estagio1 = True
estagio2 = False
estagio3 = False
estagio_creeper = False
pode  = True
ultima_placa = 0


# # Apresentações
# print("\n              Insper")
# time.sleep(1)
# print("     Engenharia da Computação")
# time.sleep(1)
# print("   Robótica Computacional 2021.1")
# time.sleep(2)

# print(Fore.RED + "\nBem vindo/a ao nosso projeto!")
# time.sleep(2)
# print(Fore.WHITE + "Bora conhecer quem gastou muito tempo das suas vidas programando?")
# time.sleep(2)
# print("               .")
# time.sleep(1)
# print("                 .")
# time.sleep(1)
# print("                   .")
# time.sleep(1)
# print("Lesgooo")
# time.sleep(2)

# print(Fore.YELLOW + "\nEstrelando...")
# time.sleep(1)
# print(Fore.WHITE + "Fernando Peres Marques Gameiro França " + Fore.YELLOW + "(aka Françinha)")
# time.sleep(2)
# print(Fore.WHITE + "Luiza Valezim Augusto Pinto " + Fore.YELLOW + "(aka A Perfeita)")
# time.sleep(2)
# print(Fore.WHITE + "Vinicius Grando Eller " + Fore.YELLOW + "(aka Sampas, mas não é o Bot Sampas... longa história)")
# time.sleep(2)


# print(Fore.GREEN + "\nFeat:")
# time.sleep(1)
# print(Fore.WHITE + "Fábio Miranda " + Fore.GREEN + "(aka Mirandinha ou Miras)")
# time.sleep(2)
# print(Fore.WHITE + "Arnaldo Junior " + Fore.GREEN + "(aka Tiozão)")
# time.sleep(2)
# print(Fore.WHITE + "Diego " + Fore.GREEN + "(aka Diego) \n\n" + Fore.WHITE)
# time.sleep(2)

# # Input
# print("Então vamos lá:")
# time.sleep(2)
# define_id = input("Qual o id que você quer? (22, sim, só tem essa opção) ")
# define_cor = input(str("Qual cor você quer? (ciano, verde, vermelho) "))
define_cor = "verde"
define_id = 21

bridge = CvBridge()

# DECLARANDO FUNÇÕES

def main():
    global robot
    global creeper
    global estagio1
    global estagio2
    global estagio3
    global estagio_creeper
    global ultima_placa
    global pode

    media, centro, maior_area = creeper.identify_color(robot)
    ids, _ = robot.getAruco()
    M = robot.get_yellow_moments()

    # Verificando cor e ID dos creepers (ainda sem a parte das cores junto)
    # try:
    #     for i in ids:
    #         if i == int(define_id) and maior_area > 1500 and pode:
    #             estagio1 = False
    #             estagio2 = False
    #             estagio3 = False
    #             estagio_creeper = True
    # except Exception:
    #     pass
    
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

        robot.follow_yellow_line()
        
        if M['m00'] <= 0:
            estagio1, estagio2 = False, True
    
    elif estagio2:

        print (" Estágio 2")

        robot.rotate()

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

        robot.follow_yellow_line(img=img)

        if M['m00'] <= 0:
            estagio1, estagio2 = False, True

    # elif estagio_creeper:

    #     print ("Estágio Creeper")
    #     bateu = robot.go_to_creeper(creeper)

    #     if bateu:
    #         estagio_creeper, estagio2 = False, True
    #         pode = False

    cv2.imshow("Main", robot.getImage(original=True))


if __name__=="__main__":

    rospy.init_node("q3")

    robot = Robot()
    creeper = Creeper(define_cor, define_id)

    rospy.sleep(0.1)        
    
    try:
        while not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        print("  Já entendi... Já pode parar de dar CTRL+C xuxu")