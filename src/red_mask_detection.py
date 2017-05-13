#!/usr/bin/env python

#
# This script reads images from a ROS topic,
# converts them and elaborates to detect red and 
# green LEDs (semaphore), eventually sending 
# encoded strings to the redmask_detection_topic topic
#
# script by Andrea Fioroni - andrifiore@gmail.com
# GitHub repo: https://github.com/isarlab-department-engineering/ros-traffic-light-detection
#

from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class traffic_light:

    def __init__(self):
      self.bridge = CvBridge() # setup CVbridge to convert from ImageMessage to CVimage
      self.image_sub = rospy.Subscriber("image_topic", Image, self.callback) # subscribe to image_topic topic
      self.imagePub = rospy.Publisher("red_mask_topic", Image, queue_size=1)	
      self.controlPub = rospy.Publisher("redmask_detection_topic", String, queue_size=10)

      # semaphore variables
      self.redSemaphore = 0
      self.greenSemaphore = 0

      ## color boundaries (bgr format) -> RED
      self.Rlower = [40, 40, 160] # lower boundaries
      self.Rupper = [120, 120, 230] # upper boundaries
      self.Rlower = np.array(self.Rlower, dtype = "uint8")
      self.Rupper = np.array(self.Rupper, dtype = "uint8")

      ## color boundaries (bgr format) -> GREEN
      self.Glower = [20, 200, 20] # lower boundaries
      self.Gupper = [50, 255, 50] # upper boundaries
      self.Glower = np.array(self.Glower, dtype = "uint8")
      self.Gupper = np.array(self.Gupper, dtype = "uint8")

      # rows and columns boundaries
      ## suppose you will always find the semaphore
      ## on the top right corner of the image (image will be 160x112)
      self.imin = 0
      self.imax = 25
      self.jmin = 60
      self.jmax = 100
      self.step = 1
	
    def callback(self,data): # runs everytime an ImageMessage is uploaded on the topic
      try:
        cvImage = self.bridge.imgmsg_to_cv2(data) # convert image to CV image
	cvImage = cv2.resize(cvImage, (160,112)) # resize the image
		
        rospy.loginfo(rospy.get_caller_id() + " Received an image")
	
	# debug code #####
	#height, width, channels = cvImage.shape
	#rospy.loginfo("IMAGE SIZE: %d x %d",height,width)  
	
	# color detection
        # generate green and red masks and use them to filter the image
        # redMask / greenMask images will display only red / green pixels
        mask = cv2.inRange(cvImage, self.Rlower, self.Rupper) # red boundaries mask
        redMask = cv2.bitwise_and(cvImage, cvImage, mask = mask) # red filtered image
        mask = cv2.inRange(cvImage, self.Glower, self.Gupper) # green boundaries mask
        greenMask = cv2.bitwise_and(cvImage, cvImage, mask = mask) # green filtered image
	

        # wait green semaphore to restart
        if self.redSemaphore == 1: # look for green semaphore only after finding a red one
            for i in range(self.imin, self.imax, self.step*2):
                for j in range(self.jmin, self.jmax, self.step*2):
                    if greenMask[i][j][1] > 0: # if that pixel is green
                        self.redSemaphore = 0 # reset redSemaphore
                        self.greenSemaphore = 1 # found a greenSemaphore
        if self.greenSemaphore == 1:
            #rospy.loginfo("FOUND GREEN")
            self.controlPub.publish("GGG") # send green encoded string to traffic_light_detection
	    self.greenSemaphore = 0

	self.redSemaphore = 0 # reset redSemaphore each time
        for i in range(self.imin, self.imax, self.step): # look for a red semaphore
            for j in range(self.jmin, self.jmax, self.step):
                if redMask[i][j][2] > 0: # if that pixel is red
                    self.redSemaphore = 1 # found a redSemaphore
        if self.redSemaphore == 1:
            rospy.loginfo("FOUND RED")
            self.controlPub.publish("RRR") # send red encoded string to traffic_light_detection

        # redundant check: everytime, if no redSemaphore is found (and not even a green one)
        # send green encoded string to rospibot_network
        if self.redSemaphore == 0:
            self.controlPub.publish("GGG")
	
	cv2.rectangle(redMask, (self.jmin,self.imin), (self.jmax,self.imax), (30,30,255), 1)
	self.imagePub.publish(self.bridge.cv2_to_imgmsg(redMask)) # publish the red masked image on the red_mask_topic topic	
	
      except CvBridgeError as e:
        print(e)


def main(args):
    ic = traffic_light()
    rospy.init_node('red_mask_detection', anonymous=True) # start a traffic_light node
    try:
        rospy.spin() # loop until shutdown
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)


