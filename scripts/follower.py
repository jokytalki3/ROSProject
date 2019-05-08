#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('robot_project')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("camera/rgb/image_color",Image,queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber('raspicam_node/image',Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
  
    # Convert input image to HSV
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    if something == "G":
      # Threshold the HSV image, keep only yellow
      lower_yellow = np.array([40,40,40])
      upper_yellow = np.array([70,255,255])
      mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    elif something == "Y":
      lower_yellow = np.array([20,100,100])
      upper_yellow = np.array([30,255,255])
      mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    elif something == "R":
      lower_yellow = np.array([0,100,100])
      upper_yellow = np.array([10,255,255])
      mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
   
    cv2.imshow("Image window", mask)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('follower', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    print("What colour? (G/Y/R)")
    something = raw_input()
    main(sys.argv)
