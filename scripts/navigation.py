#!/usr/bin/env python

# code to subscribe to image messages that is published to camera/rgb/image_raw

import rospy
import actionlib
import os
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, Twist, Quaternion, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from math import atan2,radians, pi

global goal
goal = Point()
x = 0.0
y = 0.0
theta = 0.0

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rospy.loginfo(msg.pose.pose)

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def color(data):
     
     if(data.data=='G'):
         #goal.x = -0.06
         #goal.y = -0.3
         goal.x = 2.2234
         goal.y = -0.5765
     elif(data.data=='Y'):
         
         goal.x = 2.2497
         goal.y = 0.00155
         rospy.loginfo('got yellow')
     elif(data.data=='R'):
         
         goal.x = 2.1778
         goal.y = 0.5898
     
     if  x > (goal.x-0.03) and x< (goal.x+0.03) and y > (goal.y-0.03) and y< (goal.y+0.03):
          subc.unregister()
          goal.x = 0
          goal.y = 0
          os.system('python backstart.py')
          rospy.signal_shutdown('done goes to the object')
    

def main():
     rospy.init_node('goals_destination',anonymous=True)
     sub = rospy.Subscriber('/odom', Odometry, newOdom)
     global subc
     subc = rospy.Subscriber('Color', String, color)
     pub = rospy.Publisher('/cmd_vel', Twist , queue_size=1)
     #rospy.spin()

     speed = Twist()

     r = rospy.Rate(20)
     #goal = Point()
     #goal.x = 5
     #goal.y = 5

     while not rospy.is_shutdown():
          inc_x = goal.x - x
          inc_y = goal.y - y

          angle_to_goal = atan2 (inc_y, inc_x)

          if abs(angle_to_goal - theta) > 0.15 :
               speed.linear.x = 0.0
               speed.angular.z = 0.4
          else:
               speed.linear.x = 0.5
               speed.angular.z = -0.1
          if  x > (goal.x-0.03) and x< (goal.x+0.03) and y > (goal.y-0.03) and y< (goal.y+0.03):
               goal.x = 0
               goal.y = 0
              
               

          
	  rospy.loginfo(goal)
          rospy.loginfo('in navigtion now')
          
          
          pub.publish(speed)

          
          r.sleep()

if __name__ == "__main__":
     main()
