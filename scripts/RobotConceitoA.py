import rospy 
import numpy as np
import cv2
import cv2.aruco as aruco
import statsmodels.api as sm
import math

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from subprocess import call
from colorama import Fore, Back, Style
from colorama import init
from std_msgs.msg import Float64
import mobilenet_simples as mnet
from funcoesPD import *

bridge = CvBridge()

class Robot:
    def __init__(self):
        """
        Inicializa variaveis usadas pelo robo
        """
        self.velocity = Twist()
        self.image = None
        self.image_height = None
        self.image_width = None
        self.ranges = None
        self.braco = Float64()
        self.braco.data = -0.5
        self.garra = Float64()
        self.garra.data = -1

        
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.vel_publisher = rospy.Publisher(
            '/cmd_vel',
            Twist, 
            queue_size=3)
        self.laser_subscriber = rospy.Subscriber(
            '/scan', 
            LaserScan, 
            self.laser_callback)
        self.image_subscriber = rospy.Subscriber(
            "/camera/image/compressed", 
            CompressedImage, 
            self.image_callback, 
            queue_size=4, 
            buff_size = 2**24)
        self.braco_publisher = rospy.Publisher(
            '/joint1_position_controller/command',
            Float64,
            queue_size=1)
        self.garra_publisher = rospy.Publisher(
            '/joint2_position_controller/command',
            Float64,
            queue_size=1)
    
    def detectMobileNet(self):
        """
        Detecta a MobileNet na visão do robô 
        :return result_tuples: tupla dos pesos encontrados
        """
        result_frame, result_tuples = mnet.detect(self.getImage())
        print(result_tuples)
        return result_tuples
    
    def laser_callback(self, dado):
        """
        Define as distâncias do robô até as estações
        :param dado: distancia extraidas pelo sensor
        """
        ranges = np.array(dado.ranges).round(decimals=2)
        self.setRanges(ranges)
    
    def image_callback(self, imagem):
        """
        Capta a imagem e transforma em formato que possa ser usado por CV2
        :param imagem: imagem a ser analisada 
        """
        try:
            cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")

            self.setImage(cv_image)
            
            cv2.waitKey(1)
        except CvBridgeError as e:
            print('ex', e)
    
    def publish_joints(self):
        """
        Publica os estados do braco/garra do robô  (inclinacao e aberto/fechado)
        """
        self.braco_publisher.publish(self.braco)
        self.garra_publisher.publish(self.garra)
    
    def publish_vel(self):
        """
        Publica as velocidades
        """
        self.vel_publisher.publish(self.velocity)
    
    """get e setter para velocidade"""
    def getVelocity(self):
        return self.velocity
    def setVelocity(self, vel_lin, vel_ang):
        self.velocity.linear.x = vel_lin
        self.velocity.angular.z = vel_ang
    
    """get e setter para estados do braco/garra do robô """
    def getBracoGarra(self):
        return self.braco.data, self.garra.data
    def setBracoGarra(self, braco, garra):
        self.braco.data = braco
        self.garra.data = garra
    
    """get e setter das distâncias encontradas pelo sensor"""
    def getRanges(self, angle = None):
        if angle != None:
            return self.ranges[angle]
        return self.ranges    
    def setRanges(self, new_ranges):
        self.ranges = new_ranges
    
    """get e setter das imagens, pode transforma-las em varias mascaras diferentes (gray, hsv, rgb), depende do uso"""
    def setImage(self, new_image):
        self.image = new_image
        h, w, d = new_image.shape
        self.image_height = h
        self.image_width = w
    def getImage(self, gray = False, rgb = False, hsv = False, original = False):
        if gray:
            return cv2.cvtColor(self.image.copy(), cv2.COLOR_BGR2GRAY)
        elif rgb:
            return cv2.cvtColor(self.image.copy(), cv2.COLOR_BGR2RGB)
        elif hsv:
            return cv2.cvtColor(self.image.copy(), cv2.COLOR_BGR2HSV)
        elif original:
            return self.image

        return self.image.copy()
    
    def getWidth(self):
        """
        Retorna a largura da imagem
        """
        return self.image_width
    
    def getHeight(self):
        """
        Retorna a altura da imagem
        """
        return self.image_height
    
    def getCenter(self):
        """
        Retorna as cordenadas do centro da imagem
        """
        return (self.getWidth() // 2, self.getHeight() // 2)
    
    def getAruco(self, draw = False):
        """
        Utilizada para operar com o aruco, pode retornar a imagem com os ids marcados
        :param draw: booleano que aponta se o quadrado que contorna o aruco aparecera na imagem final
        :return img: retorna a imagem com ou sem o desenho
        :return ids: dicionario de dicionarios - armazena todos os ids encontrados na imagem 
        """
        corners, ids, rejectedImgPoints = aruco.detectMarkers(self.getImage(gray=True), self.aruco_dict)
        img = self.getImage()

        if draw:
            aruco.drawDetectedMarkers(img, corners, ids)
        
        ids = np.array(ids).flatten()

        return ids, img

    def get_yellow_moments(self, img = None):
        """
        Retorna os momentos da máscara, para futuro calculo do centro de massa de um objeto
        :return M: momentos de um objeto
        """
        if img is None:
            img = self.getImage(hsv=True)


        mask = segment_yellow_line(img)
        mask = cv2.cvtColor(mask, cv2.COLOR_HSV2RGB)
        mask = cv2.cvtColor(mask, cv2.COLOR_RGB2GRAY)
        M = cv2.moments(mask)

        return M
    
    def get_PDconstants(self, img):
        # mask = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        # mask = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        gray = img.copy()
        x, y = ajuste_linear_grafico_x_fy(gray)
        xi, xf = x
        yi, yf = y

        dy = yf - yi
        hip = ((xf-xi)**2 + (dy)**2)**0.5
        seno = dy/hip
        return seno


    def follow_yellow_line(self, img = None):
        """
        Ajusta a velocidade linear para que o robô siga a pista, em retas sendo mais rapido e curvas sendo mais devagar
        :param min_vel_linear: velocidade minima nas curvas 
        :param max_vel_linear: velocidade maxima nas retas 
        """
        if img is None:
            img = self.getImage(hsv=True)


        M = self.get_yellow_moments(img = None)
        
        img = self.getImage(hsv=True)
        seno = self.get_PDconstants(img)

        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            cv2.circle(self.getImage(original=True), (cx, cy), 10, (0,0,255), -1)

            err = cx - self.getWidth()/2

            vel_linear = 0.75*seno
            vel_ang = (-float(err) / 100)*seno

            self.setVelocity(vel_linear, vel_ang)

        self.publish_vel()

    def rotate(self, vel_ang = math.pi/10, clockwise = False):
        """
        Ajusta a velocidade angular para que o robô sempre volte a seguir a pista
        :param clockwise: define se o robo gira em sentido horario ou anti
        :param vel_ang: define a velocidade com que o robo rotacionara
        """
        if clockwise:
            self.setVelocity(0, -vel_ang)
        else:
            self.setVelocity(0, vel_ang)
        
        self.publish_vel()
    
    def go_to_creeper(self, creeper, min_distance = 0.3):
        """
        Ajusta o centro do robô para alinhar e ir na direção do creeper
        :param creeper: objeto da classe creeper
        :param min_distancia: minima distancia para que o robo para antes de bater no creeper
        """
        media, centro, maior_area = creeper.identify_color(self)

        crashed = True if self.getRanges(0) < min_distance else False

        if len(media) != 0 and len(centro) != 0:
            if abs(media[0] - centro[0]) > 50:
                if (media[0] > centro[0]):
                    self.setVelocity(0, -0.3)
                if (media[0] < centro[0]):
                    self.setVelocity(0, 0.3)
            else:
                if not crashed:
                    if (media[0] > centro[0]):
                        self.setVelocity(0.1, -0.1)
                    if (media[0] < centro[0]):
                        self.setVelocity(0.1, 0.1)
        
        if crashed:
            self.setVelocity(0, 0)
            self.setBracoGarra(1.5, 0)

        self.publish_vel()

        return crashed
    
    def go_to_station(self, estacao, resultado_mobile, min_distance = 0.5):
        """
        Muda o estado do robô para ele ir em direção da estação definida
        :param estacao: mostra a estacao que o robo deve focar em encontrar
        :resultado_mobile: tupla com os pesos da mobile
        :min_distance: minima distancia para arremessar o creeper 
        """
        centro = self.getCenter()
        centro_estacao = None

        crashed = True if self.getRanges(0) < min_distance else False


        for each in resultado_mobile:
            if each[0] == estacao:
                centro_estacao = ((each[3][0] - each[2][0]) // 2) + each[2][0]
                break
        
        if centro_estacao is not None:
            if abs(centro_estacao - centro[0]) > 50:
                if (centro_estacao > centro[0]):
                    self.setVelocity(0, -0.3)
                if (centro_estacao < centro[0]):
                    self.setVelocity(0, 0.3)
            else:
                if not crashed:
                    if (centro_estacao > centro[0]):
                        self.setVelocity(0.1, -0.1)
                    if (centro_estacao < centro[0]):
                        self.setVelocity(0.1, 0.1)
            
            if crashed:
                self.setVelocity(0, 0)
                self.setBracoGarra(0, -1)

        self.publish_vel()

        return crashed