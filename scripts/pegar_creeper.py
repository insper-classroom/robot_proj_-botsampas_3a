#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division


# Para rodar, recomendamos que faça:
# 
#    roslaunch my_simulation pista_u.launch
#
# Depois o controlador do braço:
#
#    roslaunch mybot_description mybot_control2.launch 	
import rospy 

import numpy as np

import cv2
import cv2.aruco as aruco

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import statsmodels.api as sm

import math


ranges = None
estagio1 = True
estagio2 = False
distancia_passou = False 
minv = 0
maxv = 10
vel_linear = 0.1
vel_ang = 0
vel = Twist(Vector3(vel_linear,0,0), Vector3(0,0,vel_ang))

bridge = CvBridge()

def segmenta_linha_amarela_hsv(hsv):
    """ REturns a mask within the range"""
    low = np.array([25, 150, 150])
    high = np.array([35, 255, 255])
    mask = cv2.inRange(hsv, low, high)
    return mask

def morpho_limpa(mask):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(6,6))
    mask = cv2.morphologyEx( mask, cv2.MORPH_OPEN, kernel )
    mask = cv2.morphologyEx( mask, cv2.MORPH_CLOSE, kernel )    
    return mask


def center_of_mass(mask):
    """ Retorna uma tupla (cx, cy) que desenha o centro do contorno"""
    M = cv2.moments(mask)
    # Usando a expressão do centróide definida em: https://en.wikipedia.org/wiki/Image_moment
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    return [int(cX), int(cY)]

def processa_imagem(image):
    global vel
    global vel_linear
    global vel_ang
    global estagio1
    global estagio2

    if estagio1:
        img = image.copy()
        img2 = image.copy()

        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(image.copy(), cv2.COLOR_BGR2GRAY)
        mask = segmenta_linha_amarela_hsv(img)
        mask = morpho_limpa(mask)

        M = cv2.moments(mask)
        h, w, d = image.shape

        # aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        # # parameters  = aruco.DetectorParameters_create()
        # # parameters.minDistanceToBorder = 0
        # # parameters.adaptiveThreshWinSizeMax = 1000

        # corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)
        # print(ids)
        # aruco.drawDetectedMarkers(image, corners, ids)

        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 10, (0,0,255), -1)

            err = cx - w/2
            
            # Diminuir velocidade em curvas
            if abs(err) > 10:
                vel_linear = 0.2
            else:
                vel_linear = 0.5

            vel_ang = -float(err) / 100
        
        
        # try:
        #     for i in np.squeeze(ids):
        #         if i in [50, 150]:
        #             if ranges[0] < 1:
        #                 estagio1, estagio2 = False, True
        # except Exception as e:
        #     pass

    elif estagio2:
        vel_lin = 0
        vel_ang = 0


def 
    

def scaneou(dado):
    global ranges
    global minv
    global maxv
    global distancia_passou
    print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
    print("Leituras:")
    ranges = np.array(dado.ranges).round(decimals=2)
    distancia = ranges[0]
    minv = dado.range_min 
    maxv = dado.range_max

    if distancia < 0.3:
        distancia_passou = True
    else:
        distancia_passou = False
 
# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        processa_imagem(cv_image)
        cv2.imshow("Camera", cv_image)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)

if __name__=="__main__":

    rospy.init_node("q3")

    topico_imagem = "/camera/image/compressed"
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

    while not rospy.is_shutdown():
        if distancia_passou:
            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
        else:
            vel = Twist(Vector3(10,0,0), Vector3(0,0,0))
        if len(media) != 0 and len(centro) != 0:
            print("Média dos vermelhos: {0}, {1}".format(media[0], media[1]))
            print("Centro dos vermelhos: {0}, {1}".format(centro[0], centro[1]))

            if abs(media[0] - centro[0]) > 50:
                if (media[0] > centro[0]):
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,-velocidade_angular))
                if (media[0] < centro[0]):
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,velocidade_angular))
            else:
                if not distancia_passou:
                    if (media[0] > centro[0]):
                        vel = Twist(Vector3(velocidade_linear,0,0), Vector3(0,0,-velocidade_angular))
                    if (media[0] < centro[0]):
                        vel = Twist(Vector3(velocidade_linear,0,0), Vector3(0,0,velocidade_angular))
            
        velocidade_saida.publish(vel)
        vel = Twist(Vector3(vel_linear,0,0), Vector3(0,0,vel_ang))
        velocidade_saida.publish(vel)
        rospy.sleep(0.1)
