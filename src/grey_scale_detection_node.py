#!/usr/bin/env python

#
# This script reads images from a ROS topic,
# converts them and elaborates to detect light peaks 
# from greyscale image, eventually sending 
# encoded strings to the grey_scale_topic topic
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
      self.controlPub = rospy.Publisher('greyscale_detection_topic', String, queue_size=10) # publish on rospibot_network topic
      self.greyPub = rospy.Publisher("grey_scale_topic", Image, queue_size=10)	

      # semaphore variables
      self.redSemaphore = 0
      self.greenSemaphore = 0

      ## color boundaries (greyscale image)
      self.lower = [210] # lower boundary
      self.upper = [230] # upper boundary
      self.lower = np.array(self.lower, dtype = "uint8")
      self.upper = np.array(self.upper, dtype = "uint8")

      # rows and columns boundaries
      ## suppose you will always find the semaphore
      ## on the top right corner of the image (image will be 160x112)
      self.imin = 0
      self.imax = 25
      self.jmin = 100
      self.jmax = 155
      self.step = 2

    def callback(self,data): # runs everytime an ImageMessage is uploaded on the topic
      try:
        cvImage = self.bridge.imgmsg_to_cv2(data) # convert image to CV image
	cvImage = cv2.resize(cvImage, (160,112)) # resize the image
	cvImage = cv2.cvtColor(cvImage, cv2.COLOR_BGR2GRAY)
        rospy.loginfo(rospy.get_caller_id() + " Received an image")
	
	# color detection
        # generate greyscale mask and use it to filter the image
        # greyMask image will display only high brightness pixels
        mask = cv2.inRange(cvImage, self.lower, self.upper) # greyscale boundaries mask
        greyMask = cv2.bitwise_and(cvImage, cvImage, mask = mask) # brightness filtered image

        # wait green semaphore to restart
        #if self.redSemaphore == 1: # look for green semaphore only after finding a red one
        #    for i in range(self.imin, self.imax, self.step*2):
        #        for j in range(self.jmin, self.jmax, self.step*2):
        #            if greyMask[i][j] > 0: # if that pixel is bright
        #                self.redSemaphore = 0 # reset redSemaphore
        #                self.greenSemaphore = 1 # found a greenSemaphore
        #if self.greenSemaphore == 1:
        #    rospy.loginfo("NO LIGHT FOUND")
        #    self.controlPub.publish("GGG") # send green encoded string to traffic_light_detection	
	#    self.greenSemaphore = 0

	self.redSemaphore = 0 # reset redSemaphore each time
        for i in range(self.imin, self.imax, self.step): # look for a red semaphore
            for j in range(self.jmin, self.jmax, self.step):
                if greyMask[i][j] > 0: # if that pixel is bright
                    self.redSemaphore = 1 # found a redSemaphore
        if self.redSemaphore == 1:
            rospy.loginfo("FOUND LIGHT")
            self.controlPub.publish("RRR") # send red encoded string to rospibot_network

        # redundant check: everytime, if no redSemaphore is found
        # send green encoded string to topic
        if self.redSemaphore == 0:
            self.controlPub.publish("GGG")
	    rospy.loginfo("NO LIGHT FOUND")		
	
	cv2.rectangle(greyMask, (self.jmin,self.imin), (self.jmax,self.imax), (255), 1)	
	self.greyPub.publish(self.bridge.cv2_to_imgmsg(greyMask)) # publish the grayscale masked image on the grey_scale_topic

      except CvBridgeError as e:
        print(e)


def main(args):
    ic = traffic_light()
    rospy.init_node('grey_scale_detection', anonymous=True) # start a traffic_light node
    try:
        rospy.spin() # loop until shutdown
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)


