#!/usr/bin/env python

# ------------------------------------------------------------------------------
# Filtro color con seguimiento del centro para topic de imagen ROS
# Autor: Diego Martin. Filtro color adaptado de https://www.pyimagesearch.com/)
# Fecha: 08/02/2020
# ------------------------------------------------------------------------------

import numpy as np
import imutils
import time
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Define input topic and ball boundaries in HSV
input_image_topic="/camera/rgb/image_raw"
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)

class image_converter_filter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/image_filtered",Image,queue_size = 1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(input_image_topic,Image,self.callback_filter_opencv,queue_size = 1) 

  def callback_filter_opencv(self,data):
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	# resize the frame, blur it, and convert it to HSV
	frame = imutils.resize(cv_image, width=600)
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	center = None

	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		
		# only proceed if the radius meets a minimum size
		if radius > 10:
			
			# draw the circle (yellow) and its center (red) on the frame, print center
			cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)
			print "CENTER COORDINATES=", int(x), int(y)

	# show the image with the circle/center
	cv2.imshow("Filtered_image", frame)
	cv2.waitKey(3) # needed to update the imshow
  
    except CvBridgeError as e:
      print(e)
 
    #Publish the filtered image in ROS
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter_filter()
  rospy.init_node('image_converter_filter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
