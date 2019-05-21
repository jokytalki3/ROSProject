#!/usr/bin/env python

# code to subscribe to image messages that is published to camera/rgb/image_raw

import rospy
import actionlib
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, Twist, Quaternion, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from math import atan2,radians, pi

x = 0.0
y = 0.0
theta = 0.0

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node('goals_destination')
sub = rospy.Subscriber('/odom', Odometry, newOdom)
pub = rospy.Publisher('/cmd_vel', Twist , queue_size=1)
#rospy.spin()

speed = Twist()

r = rospy.Rate(4)

goal = Point()
goal.x = 5
goal.y = 5

while not rospy.is_shutdown():
     inc_x = goal.x - x
     inc_y = goal.y - y

     angle_to_goal = atan2 (inc_y, inc_x)

     if abs(angle_to_goal - theta) > 0.1 :
          speed.linear.x = 0.0
          speed.angular.z = 0.3
     else:
          speed.linear.x = 0.5
          speed.angular.z = 0.0

     pub.publish(speed)
     r.sleep()
