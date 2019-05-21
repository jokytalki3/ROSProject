#!/usr/bin/env python


from __future__ import print_function

import roslib
roslib.load_manifest('robot_project')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from collections import deque
from geometry_msgs.msg import Twist
msg = Twist()

class image_converter:

  def __init__(self): 

    
 
    #self.image_pub = rospy.Publisher("camera/rgb/image_color",Image,queue_size=10)
    self.pub = rospy.Publisher("coordinates",Float32,queue_size=1)
    self.pubx = rospy.Publisher("coordinatesx", Float32, queue_size=1)
    self.pubr = rospy.Publisher("circleRadius", Float32, queue_size=1)
    self.pubg = rospy.Publisher("Color" , String, queue_size=1)


    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber('raspicam_node/image',Image,self.callback)
    
    
    #self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
  
    
    # Convert input image to HSV
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    if something == "G":
      # Threshold the HSV image, keep only yellow
      lower_yellow = np.array([29, 100, 29])
      upper_yellow = np.array([100,255,100])
      #lower_yellow = np.array([40,40,40])
      #upper_yellow = np.array([70,255,255])
      mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    elif something == "Y":
      lower_yellow = np.array([25,50,50])
      upper_yellow = np.array([32,255,255])
      mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    elif something == "R":
      lower_yellow = np.array([161, 155, 84])
      upper_yellow = np.array([179,255,255])
      mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    pts = deque(maxlen=64)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    if len(cnts) > 0:
	c = max(cnts, key=cv2.contourArea)
	((x, y), radius) = cv2.minEnclosingCircle(c)
	M = cv2.moments(c)
	center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	#print(center)
	self.pub.publish(int(M["m01"] / M["m00"]))
        if radius < 300:
             self.pubx.publish(int(M["m10"] / M["m00"]))
        
        self.pubr.publish(int(radius))
	self.pubg.publish(something)
	rospy.loginfo(int(M["m01"] / M["m00"]))

	if radius > 10:
		cv2.circle(cv_image, (int(x), int(y)), int(radius),(0, 255, 255), 2)
		cv2.circle(cv_image, center, 5, (0, 0, 255), -1)
    pts.appendleft(center)
    for i in xrange(1, len(pts)):
	    if pts[i - 1] is None or pts[i] is None:
		    continue
	    thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
	    cv2.line(cv_image, pts[i - 1], pts[i], (0, 0, 255), thickness)
   
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    #try:
      #self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
   # except CvBridgeError as e:
      #print(e)

def main(args):
  rospy.init_node('follower', anonymous=True)
  ic = image_converter()
  

  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

 

if __name__ == '__main__':
    print("What colour? (G/Y/R)")
    something = raw_input()
    main(sys.argv)

