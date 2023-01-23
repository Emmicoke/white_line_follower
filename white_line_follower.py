#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
from cart_sim.msg import cart_control
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class Linedetection():
    def __init__(self):
        rospy.init_node("line_detection")
        self.bridge = CvBridge()
        rospy.Subscriber("/cart/front_camera/image_raw",Image,self.cameraCallback)
        #self.pub = rospy.Publisher("cmd_vel",Twist,queue_size=10)
        self.speed_message = Twist()
        self.pub = rospy.Publisher('cart', cart_control, queue_size=1)
        rospy.spin()

    def get_vertices(image):
        rows, cols = image.shape[:2]
        bottom_left  = [cols*0.15, rows]
        top_left     = [cols*0.45, rows*0.6]
        bottom_right = [cols*0.95, rows]
        top_right    = [cols*0.55, rows*0.6] 
        
        ver = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
        return ver

    def cameraCallback(self,message):
        img = self.bridge.imgmsg_to_cv2(message,"bgr8")
        rows, cols = img.shape[:2]
        bottom_left  = [cols*0.15, rows]
        top_left     = [cols*0.45, rows*0.6]
        bottom_right = [cols*0.95, rows]
        top_right    = [cols*0.55, rows*0.6] 
        low_threshold = 180
        high_threshold = 240
        cv2.Canny(img, low_threshold, high_threshold)
            #defining a blank mask to start with
        mask = np.zeros_like(img)   
        
        #defining a 3 channel or 1 channel color to fill the mask with depending on the input image
        if len(img.shape) > 2:
            channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255
            
        vertices = Linedetection.get_vertices(img) 
        #filling pixels inside the polygon defined by "vertices" with the fill color    
        cv2.fillPoly(mask, vertices, ignore_mask_color)
        
        #returning the image only where mask pixels are nonzero
        masked_image = cv2.bitwise_and(img, mask)
        hsv = cv2.cvtColor(masked_image,cv2.COLOR_BGR2HSV)
        low_white = np.array([0,0,165])
        up_white = np.array([255,255,255])
        maske = cv2.inRange(hsv,low_white,up_white)
        sonuc = cv2.bitwise_and(masked_image, masked_image,mask=maske)
        h,w,d = masked_image.shape
        cv2.circle(masked_image,(int(w/2),int(h/2)),5,(0,0,255),-1)
        M = cv2.moments(maske)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(img,(cx,cy),5,(255,0,0),-1)
            sapma = cx - w/2
            print(sapma)
            self.speed_message.linear.x = 0.01
            self.speed_message.angular.z = -sapma/100
            speed = cart_control()
            speed.throttle = self.speed_message.linear.x
            speed.steer = self.speed_message.angular.z
            self.pub.publish(speed)
        else:
            self.speed_message.linear.x = 0.
            self.speed_message.angular.z = 0.0
            speed = cart_control()
            speed.throttle = self.speed_message.linear.x
            speed.steer = self.speed_message.angular.z
            self.pub.publish(speed)
            #self.speed_message.linear.x = 0.0
            #self.speed_message.angular.z = 0.0
            #self.pub.publish(self.speed_message)
        cv2.imshow("Orijinal",img)
        cv2.imshow("Maske",maske)
        cv2.imshow("Sonuc",sonuc)
        cv2.waitKey(1)

Linedetection()