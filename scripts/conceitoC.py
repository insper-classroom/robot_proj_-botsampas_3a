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

#--------------------------------------------------------------------------

# DECLARANDO VARIÁVEIS

ranges = None
estagio1 = True
estagio2 = False
estagio3 = False
estagio_creeper = False
pode  = True
ultima_placa = 0
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
minv = 0
maxv = 10
vel_linear = 0
vel_ang = 0
vel = Twist(Vector3(vel_linear,0,0), Vector3(0,0,vel_ang))


# Apresentações
print("\n              Insper")
time.sleep(1)
print("     Engenharia da Computação")
time.sleep(1)
print("   Robótica Computacional 2021.1")
time.sleep(2)

print(Fore.RED + "\nBem vindo/a ao nosso projeto!")
time.sleep(2)
print(Fore.WHITE + "Bora conhecer quem gastou muito tempo das suas vidas programando?")
time.sleep(2)
print("               .")
time.sleep(1)
print("                 .")
time.sleep(1)
print("                   .")
time.sleep(1)
print("Lesgooo")
time.sleep(2)

print(Fore.YELLOW + "\nEstrelando...")
time.sleep(1)
print(Fore.WHITE + "Fernando Peres Marques Gameiro França " + Fore.YELLOW + "(aka Françinha)")
time.sleep(2)
print(Fore.WHITE + "Luiza Valezim Augusto Pinto " + Fore.YELLOW + "(aka A Perfeita)")
time.sleep(2)
print(Fore.WHITE + "Vinicius Grando Eller " + Fore.YELLOW + "(aka Sampas, mas não é o Bot Sampas... longa história)")
time.sleep(2)


print(Fore.GREEN + "\nFeat:")
time.sleep(1)
print(Fore.WHITE + "Fábio Miranda " + Fore.GREEN + "(aka Mirandinha ou Miras)")
time.sleep(2)
print(Fore.WHITE + "Arnaldo Junior " + Fore.GREEN + "(aka Tiozão)")
time.sleep(2)
print(Fore.WHITE + "Diego " + Fore.GREEN + "(aka Diego) \n\n" + Fore.WHITE)
time.sleep(2)

# Input
print("Então vamos lá:")
time.sleep(2)
define_id = input("Qual o id que você quer? (22, sim, só tem essa opção) ")
define_cor = input(str("Qual cor você quer? (ciano, verde, vermelho) "))
# define_cor = "verde"
# define_id = 21

bridge = CvBridge()



# DECLARANDO FUNÇÕES

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

def processa_imagem(image):
    global vel
    global vel_linear
    global vel_ang
    global estagio1
    global estagio2
    global estagio3
    global estagio_creeper
    global aruco_dict
    global ultima_placa
    global media
    global pode

    img = image.copy()
    img2 = image.copy()

    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)

    gray = cv2.cvtColor(image.copy(), cv2.COLOR_BGR2GRAY)
    mask = segmenta_linha_amarela_hsv(img)
    mask = morpho_limpa(mask)

    M = cv2.moments(mask)
    h, w, d = image.shape

    cv_image2 = image.copy()
    media, centro, maior_area = visao_module.identifica_cor(cv_image2, define_cor)
    # cv2.imshow("Creeper", cv_image2)
    print(maior_area)

    # ARUCO
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)
    aruco.drawDetectedMarkers(image, corners, ids)


    # Verificando cor e ID dos creepers (ainda sem a parte das cores junto)
    try:
        ids = np.squeeze(ids)
        for i in ids:
            if i == int(define_id) and maior_area > 1500 and pode:
                estagio1 = False
                estagio2 = False
                estagio3 = False
                estagio_creeper = True
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
        else:
            estagio1, estagio2 = False, True
    elif estagio2:

        print (" Estágio 2")

        vel_linear = 0
        vel_ang = math.pi/10

        if M['m00'] > 0:
            estagio2, estagio3 = False, True
    elif estagio3:

        print("Estágio 3")
        
        if ultima_placa == "ESQUERDA":
            img2[:, 3*w//4:] = [0, 0, 0]
        else:
            img2[:, :w//4] = [0, 0, 0]
        
        mask = segmenta_linha_amarela_hsv(img2)
        mask = morpho_limpa(mask)

        M = cv2.moments(mask)
        h, w, d = image.shape

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
        else:
            estagio1, estagio2 = False, True
    elif estagio_creeper:

        print ("Estágio Creeper")

        vel_linear = 0
        vel_ang = 0

        bateu = True if ranges[0] < 0.3 else False

        if bateu:
            estagio_creeper, estagio2 = False, True
            pode = False
        
        if len(media) != 0 and len(centro) != 0:
            if abs(media[0] - centro[0]) > 50:
                if (media[0] > centro[0]):
                    vel_ang = -0.1
                    vel_linear = 0
                if (media[0] < centro[0]):
                    vel_ang = 0.1
                    vel_linear = 0
            else:
                if not bateu:
                    if (media[0] > centro[0]):
                        vel_ang = -0.1
                        vel_linear = 0.1
                    if (media[0] < centro[0]):
                        vel_ang = 0.1
                        vel_linear = 0.1
    



def scaneou(dado):
    global ranges
    global minv
    global maxv
    # print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
    # print("Leituras:")
    ranges = np.array(dado.ranges).round(decimals=2)
    minv = dado.range_min 
    maxv = dado.range_max

 
# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        processa_imagem(cv_image)
        # cv_image = bridge.imgmsg_to_cv2(imagem, "bgr8")
        cv2.imshow("Camera", cv_image)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)


    

if __name__=="__main__":

    rospy.init_node("q3")

    topico_imagem = "/camera/image/compressed"
    # topico_imagem2 = "/camera/image"
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    # recebedor = rospy.Subscriber(topico_imagem2, Image, roda_todo_frame, queue_size=4, buff_size = 2**24)


    try:

        while not rospy.is_shutdown():
            vel = Twist(Vector3(vel_linear,0,0), Vector3(0,0,vel_ang))
            velocidade_saida.publish(vel)
            rospy.sleep(0.1)


    except rospy.ROSInterruptException:
        print("  Já entendi... Já pode parar de dar CTRL+C xuxu")