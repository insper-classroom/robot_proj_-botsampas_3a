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
from std_msgs.msg import Float64

bridge = CvBridge()

def segment_yellow_line(hsv):
	low = np.array([25, 150, 150])
	high = np.array([35, 255, 255])
	mask = cv2.inRange(hsv, low, high)
	return mask

def morpho_clean(mask):
	kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(6,6))
	mask = cv2.morphologyEx( mask, cv2.MORPH_OPEN, kernel )
	mask = cv2.morphologyEx( mask, cv2.MORPH_CLOSE, kernel )    
	return mask

class Robot:
	def __init__(self):
		self.velocity = Twist()
		self.image = None
		self.image_height = None
		self.image_width = None
		self.ranges = None

		self.pos_braco = Float64()
		self.pos_braco.data = -0.4
		self.pos_garra = Float64()
		self.pos_garra.data = -1
		
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
		


	# movimentacao da garra
	def move_joints_init(self):
		
		
		self.braco_publisher.publish(pos_braco)
		self.garra_publisher.publish(pos_garra)
	
	def move_joints(self, braco, garra):
		pos_braco = Float64()
		pos_braco.data = braco
		pos_garra = Float64()
		pos_garra.data = garra
		
		self.braco_publisher.publish(pos_braco)
		self.garra_publisher.publish(pos_garra)

	##################################
	def laser_callback(self, dado):
		ranges = np.array(dado.ranges).round(decimals=2)
		self.setRanges(ranges)
	
	def image_callback(self, imagem):
		try:
			cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")

			self.setImage(cv_image)
			
			cv2.waitKey(1)
		except CvBridgeError as e:
			print('ex', e)
	
	
	def publish_in_cmd_vel(self):
		self.vel_publisher.publish(self.velocity)
	
	def getVelocity(self):
		return self.velocity
	
	def setVelocity(self, vel_lin, vel_ang):
		self.velocity.linear.x = vel_lin
		self.velocity.angular.z = vel_ang
	
	def getRanges(self, angle = None):
		if angle != None:
			return self.ranges[angle]
		return self.ranges
	
	def setRanges(self, new_ranges):
		self.ranges = new_ranges
	
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
		return self.image_width
	
	def getHeight(self):
		return self.image_height
	
	def getAruco(self, draw = False):
		corners, ids, rejectedImgPoints = aruco.detectMarkers(self.getImage(gray=True), self.aruco_dict)
		img = self.getImage()

		if draw:
			aruco.drawDetectedMarkers(img, corners, ids)
		
		ids = np.array(ids).flatten()

		return ids, img

	def get_yellow_moments(self, img = None):

		if img is None:
			img = self.getImage(hsv=True)

		mask = segment_yellow_line(img)
		mask = morpho_clean(mask)

		M = cv2.moments(mask)

		return M


	def follow_yellow_line(self, min_vel_linear = 0.2, max_vel_linear = 0.5, img = None):

		if img is None:
			img = self.getImage(hsv=True)

		M = self.get_yellow_moments(img)

		if M['m00'] > 0:
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])

			cv2.circle(self.getImage(original=True), (cx, cy), 10, (0,0,255), -1)

			err = cx - self.getWidth()/2
			
			if abs(err) > 10:
				vel_linear = min_vel_linear
			else:
				vel_linear = max_vel_linear

			vel_ang = -float(err) / 100

			self.setVelocity(vel_linear, vel_ang)

		self.publish_in_cmd_vel()

	
	def rotate(self, vel_ang = math.pi/10, clockwise = False):
		if clockwise:
			self.setVelocity(0, -vel_ang)
		else:
			self.setVelocity(0, vel_ang)
		
		self.publish_in_cmd_vel()
	
	def go_to_creeper(self, creeper, min_distance = 0.19):

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
			self.move_joints(1.5, 0)

		self.publish_in_cmd_vel()

		return crashed

	