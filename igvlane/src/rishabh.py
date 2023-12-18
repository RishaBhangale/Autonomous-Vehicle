#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math

class igvc():

	def shutdown(self):
		print("lane_daf STOP")

	def none(self,a):
		pass

	def camera(self,data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			return

	def get_hist(self,img,flag):
		x=np.sum(img,axis=1)
		y=np.sum(img,axis=0)
		try:
			x1 = np.min(np.nonzero(x))
			x2 = np.max(np.nonzero(x))
			y1 = np.min(np.nonzero(y))
			y2 = np.max(np.nonzero(y))
			return (x1,y1,x2,y2)
		except:
			if flag == 0:
				return (round(img.shape[0]*self.p),0,img.shape[0],0)
			elif flag == 1:
				return (round(img.shape[0]*self.p),img.shape[1],img.shape[0],img.shape[1])

	def __init__(self):
		rospy.init_node('lane_daf')
		print("lane_daf START")
		rospy.on_shutdown(self.shutdown)

		self.speed = 1
		self.joystick_message = Twist()
		self.joystick_message.linear.x = 0
		self.joystick_message.linear.y = 0
		self.joystick_message.linear.z = 0
		self.joystick_message.angular.x = 0
		self.joystick_message.angular.y = 0
		self.joystick_message.angular.z = 0

		self.jostick_publisher = rospy.Publisher('/joystick_data', Twist,queue_size=10)

		
		self.lj = 0 #lr
		self.rj = 0 #fwd

		self.img = None
		self.bridge = CvBridge()

		self.cam = cv2.VideoCapture(2)

		self.intialTrackBarVals = [0, 0, 193, 255, 255, 255]
		self.avg=0
		self.p=0.6

		cv2.namedWindow("Trackbars")
		cv2.resizeWindow("Trackbars", 360, 360)
		cv2.createTrackbar("H", "Trackbars", self.intialTrackBarVals[0],255,self.none)
		cv2.createTrackbar("S", "Trackbars", self.intialTrackBarVals[1],255,self.none)
		cv2.createTrackbar("V", "Trackbars", self.intialTrackBarVals[2],255,self.none)
		cv2.createTrackbar("H2", "Trackbars", self.intialTrackBarVals[3],255,self.none)
		cv2.createTrackbar("S2", "Trackbars", self.intialTrackBarVals[4],255,self.none)
		cv2.createTrackbar("V2", "Trackbars", self.intialTrackBarVals[5],255,self.none)

		#rospy.Subscriber("/camera1/image_raw", Image, self.camera)

	def lds(self):
		ret, self.img = self.cam.read()
		if ret == True:
			w=self.img.shape[1]
			h=self.img.shape[0]
			self.img = self.img[:,:int(w/2)]
			w=self.img.shape[1]
			h=self.img.shape[0]
			self.intialTrackBarVals[0] = cv2.getTrackbarPos("H", "Trackbars")
			self.intialTrackBarVals[1] = cv2.getTrackbarPos("S", "Trackbars")
			self.intialTrackBarVals[2] = cv2.getTrackbarPos("V", "Trackbars")
			self.intialTrackBarVals[3] = cv2.getTrackbarPos("H2", "Trackbars")
			self.intialTrackBarVals[4] = cv2.getTrackbarPos("S2", "Trackbars")
			self.intialTrackBarVals[5] = cv2.getTrackbarPos("V2", "Trackbars")

			hsv = cv2.cvtColor(self.img,cv2.COLOR_BGR2HSV)
			maskedWhite = cv2.inRange(hsv,np.array([self.intialTrackBarVals[0],self.intialTrackBarVals[1],self.intialTrackBarVals[2]]),np.array([self.intialTrackBarVals[3],self.intialTrackBarVals[4],self.intialTrackBarVals[5]]))
			imgBlur = cv2.GaussianBlur(maskedWhite, (5, 5), 0)

			p=0.6
			k=0.25
			img1 = cv2.line(self.img,(0,int(h*p)),(w,int(h*p)),(0,255,0),1)
			polygons = np.array([[(int(k*w),int(h*p)),(int((1-k)*w),int(h*p)),(w,h),(0,h)]])
			mask = np.zeros_like(imgBlur)
			cv2.fillPoly(mask,polygons,255)
			masked_image = cv2.bitwise_and(imgBlur,mask)
			masked_image = cv2.Canny(masked_image,50,150)

			img1 = masked_image[:,0:int(w/2)]
			img2 = masked_image[:,int(w/2):]
			x11,y11,x12,y12 = self.get_hist(img1,0)
			x21,y21,x22,y22 = self.get_hist(img2,1)

			if x11 == 0 and y11 == 0 and x12 ==0 and y12 ==0 and x21 == 0 and y21 == 0 and x22 ==0 and y22 ==0:
				print('both error')
				self.rj = 0
				self.lj = 0
				cv2.imshow('LD',self.img)
				cv2.waitKey(1)
			else:
				#print('both')
				imgg = self.img.copy()
				img1 = cv2.line(imgg[:,0:int(w/2),:],(y11,x12),(y12,x11),(255,0,0),5)
				img2 = cv2.line(imgg[:,int(w/2):,:],(y21,x21),(y22,x22),(255,0,0),5)
				finals = np.concatenate((img1, img2), axis=1)
				finals = cv2.line(finals,(y12,x11),(y21+int(w/2),x21),(255,0,0),5)
				# midptx = int((y12+y21+int(w/2))/2)
				# midpty = int((x11+x21)/2)
				# self.avg = int((midptx+self.avg)/2)
				# pp = (-(int(w/2)-self.avg)/100) #tuning
				# if pp >0.3:
				# 	pp=0.3
				# elif pp<-0.3:
				# 	pp=-0.3
				# finals = cv2.line(finals,(self.avg,(int(h/2)-20)),(self.avg,(int(h/2)+20)),(255,0,0),5)
				# finals = cv2.putText(finals,str(pp), (int(w/2),30), cv2.FONT_HERSHEY_SIMPLEX ,1, (255, 0, 0) , 2, cv2.LINE_AA)

				height = 200
				midptx1 = int((y12+y21+int(w/2))/2)
				midpty1 = int((x11+x21)/2)
				midptx2 = int(w/2)
				midpty2 = h
				finals = cv2.line(finals,(midptx1,midpty1),(midptx2,midpty2),(255,0,0),5)
				pp = (math.degrees(math.atan2((midpty2-midpty1), midptx2-midptx1)) - 90)*1
				self.avg = ((pp+self.avg)/2)
				finals = cv2.putText(finals,str(self.avg), (int(w/2),30), cv2.FONT_HERSHEY_SIMPLEX ,1, (255, 0, 0) , 2, cv2.LINE_AA)
				print(self.avg)	
				self.joystick_message.linear.x = 1
				self.joystick_message.linear.y = 0
				self.joystick_message.linear.z = self.avg/10
				self.joystick_message.angular.x = 0
				self.joystick_message.angular.y = 0
				self.joystick_message.angular.z = 0
				self.jostick_publisher.publish(self.joystick_message)
			cv2.imshow('LD',finals)
			cv2.waitKey(1)



	def loop(self):
		if self.img is not None:
			ret, self.img = self.cam.read()

			w=self.img.shape[1]
			h=self.img.shape[0]

			self.intialTrackBarVals[0] = cv2.getTrackbarPos("H", "Trackbars")
			self.intialTrackBarVals[1] = cv2.getTrackbarPos("S", "Trackbars")
			self.intialTrackBarVals[2] = cv2.getTrackbarPos("V", "Trackbars")
			self.intialTrackBarVals[3] = cv2.getTrackbarPos("H2", "Trackbars")
			self.intialTrackBarVals[4] = cv2.getTrackbarPos("S2", "Trackbars")
			self.intialTrackBarVals[5] = cv2.getTrackbarPos("V2", "Trackbars")

			hsv = cv2.cvtColor(self.img,cv2.COLOR_BGR2HSV)
			maskedWhite = cv2.inRange(hsv,np.array([self.intialTrackBarVals[0],self.intialTrackBarVals[1],self.intialTrackBarVals[2]]),np.array([self.intialTrackBarVals[3],self.intialTrackBarVals[4],self.intialTrackBarVals[5]]))
			imgBlur = cv2.GaussianBlur(maskedWhite, (5, 5), 0)
			p=0.6
			img1 = cv2.line(self.img,(0,int(h*p)),(w,int(h*p)),(0,255,0),1)
			polygons = np.array([[(0,int(h*p)),(w,int(h*p)),(w,h),(0,h)]])
			mask = np.zeros_like(imgBlur)
			cv2.fillPoly(mask,polygons,255)
			masked_image = cv2.bitwise_and(imgBlur,mask)
			masked_image = cv2.Canny(masked_image,50,150)

			img1 = masked_image[:,0:int(w/2)]
			img2 = masked_image[:,int(w/2):]
			x11,y11,x12,y12 = self.get_hist(img1)
			x21,y21,x22,y22 = self.get_hist(img2)

			if x11 == 0 and y11 == 0 and x12 ==0 and y12 ==0 and x21 == 0 and y21 == 0 and x22 ==0 and y22 ==0:
				print('both error')
				self.rj = 0
				self.lj = 0
				cv2.imshow('LD',self.img)
				cv2.waitKey(1)
			elif x11 == 0 and y11 == 0 and x12 ==0 and y12 ==0:
				print('left error')
				self.rj = 1
				self.lj = -0.6
				cv2.imshow('LD',self.img)
				cv2.waitKey(1)
			elif x21 == 0 and y21 == 0 and x22 ==0 and y22 ==0:
				print('right error')
				self.rj = 1
				self.lj = 0.6
				cv2.imshow('LD',self.img)
				cv2.waitKey(1)
			else:
				print('both')
				imgg = self.img.copy()
				depth = np.zeros_like(self.img)
				img1 = cv2.line(imgg[:,0:int(w/2),:],(y11,x12),(y12,x11),(255,0,0),5)
				img2 = cv2.line(imgg[:,int(w/2):,:],(y21,x21),(y22,x22),(255,0,0),5)
				finals = np.concatenate((img1, img2), axis=1)
				finals = cv2.line(finals,(y12,x11),(y21+int(w/2),x21),(255,0,0),5)
				height = 200
				pts_left = np.array([(y11,x12),(y12,x11),(y12,x11-height),(y11,x12-height),(y11,x12)])
				finalss = cv2.fillPoly(depth, pts=[pts_left], color=(255, 255, 255))
				pts_left = np.array([(y21+int(w/2),x21),(y22+int(w/2),x22),(y22+int(w/2),x22-height),(y21+int(w/2),x21-height)])
				finalss = cv2.fillPoly(depth, pts=[pts_left], color=(255, 255, 255))
				finalss = cv2.cvtColor(finalss,cv2.COLOR_BGR2GRAY)
				finalss = (255-finalss)
				midptx = int((y12+y21+int(w/2))/2)
				midpty = int((x11+x21)/2)
				self.avg = int((midptx+self.avg)/2)
				pp = (-(int(w/2)-self.avg)/200) #tuning
				finals = cv2.line(finals,(self.avg,(int(h/2)-20)),(self.avg,(int(h/2)+20)),(255,0,0),5)
				finals = cv2.putText(finals,str(pp), (int(w/2),30), cv2.FONT_HERSHEY_SIMPLEX ,1, (255, 0, 0) , 2, cv2.LINE_AA)
				print(pp)
				self.lj = pp
				self.rj = 1 #tuning
				#cv2.imshow('LD',finals)
				cv2.imshow('LDs',self.img)
				cv2.waitKey(1)
			self.joystick_message.linear.x = self.rj
			self.joystick_message.linear.y = 0
			self.joystick_message.linear.z = self.lj
			self.joystick_message.angular.x = 0
			self.joystick_message.angular.y = 0
			self.joystick_message.angular.z = 0
			self.jostick_publisher.publish(self.joystick_message)
			
if __name__ == '__main__':
	pub = rospy.Publisher('/float_data', Float32, queue_size=10)
	obj = igvc()
	rate = rospy.Rate(30)
	while not rospy.is_shutdown():
		obj.lds()
		rate.sleep()