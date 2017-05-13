#!/usr/bin/env python

#
# This script puts together the informations from
# redscale_detection and greyscale_detection and 
# communicates the effective presence of the 
# traffic light (when both the detections found it)
#
# script by Andrea Fioroni - andrifiore@gmail.com
# GitHub repo: https://github.com/isarlab-department-engineering/ros-traffic-light-detection
#

import rospy
import sys
from std_msgs.msg import String
import time
import atexit

class traffic_light_detection:

    def __init__(self):

	# semaphore variables
	self.redMskSemaphore = 1
	self.greyScaleSemaphore = 1
    
	self.controlPub = rospy.Publisher("traffic_light_detection", String, queue_size=10)
	rospy.Subscriber("redmask_detection_topic", String, self.callback0) # subscribe to redmask_detection topic
        rospy.Subscriber("greyscale_detection_topic", String, self.callback1) # subscribe to greyscale_detection topic
	rospy.loginfo("Listening on two different topics")

    def callback0(self,data): # runs whenever any data is published on the redmask_detection topic
        rospy.loginfo(rospy.get_caller_id() + " Getting REDMASK Info: %s", data.data)
        input = data.data # input received (will always be a string)

	if input == "GGG":
	    self.redMaskSemaphore = 0
	elif input == "RRR":
	    self.redMaskSemaphore = 1

	if self.redMaskSemaphore == 1:
	    if self.greyScaleSemaphore == 1:
		self.controlPub.publish("RRR")
		rospy.loginfo("FOUND RED")
	else:
	    self.controlPub.publish("GGG")

    def callback1(self,data): # runs whenever any data is published on the greyscale_detection topic
        rospy.loginfo(rospy.get_caller_id() + " Getting GREYSCALE Info: %s", data.data)
        input = data.data # input received (will always be a string)

	if input == "GGG":
            self.greyScaleSemaphore = 0
        elif input == "RRR":
            self.greyScaleSemaphore = 1

        if self.greyScaleSemaphore == 1:
            if self.redMaskSemaphore == 1:
                self.controlPub.publish("RRR")
		rospy.loginfo("FOUND RED")
	else:
            self.controlPub.publish("GGG")

def main(args):
    tl_det = traffic_light_detection()
    rospy.init_node('trafficlight_detection', anonymous=True) # create a trafficlight_detection node
    rospy.loginfo("started")
    try:
	rospy.spin() # loop until shutdown
    except KeyboardInterrupt:
	print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
