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


class Creeper:
    def __init__(self, color, id):
        self.color = color
        self.id = id

    def getColor(self):
        """
        Recebe a cor do creeper 
        :param self :
        :return self.color:
        """
        return self.color
    
    def getId(self):
        """
        Recebe o ID do creeper 
        :param self :
        :return self.id:
        """
        return self.id

    def identify_color(self, robot):
        """
        Identifica a média, o centro e a maior área do creeper com base na visão do robô
        :param self, robot:
        :return media, centro, maior_area:
        """
        media, centro, maior_area = visao_module.identifica_cor(robot.getImage(), self.getColor())
        return media, centro, maior_area