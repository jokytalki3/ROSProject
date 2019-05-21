#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import os
msg=Twist()
bridge = CvBridge()
radius = 0.0
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('rotate_robot', anonymous=True)   
    
    rate = rospy.Rate(10)
    msg.angular.z = 0.5
    pubz = rospy.Publisher("/cmd_vel", Twist, queue_size =1)
    #rospy.Subscriber('/baseScan', Float32, callback2)  
    global coordinatesx
    global circleRadius
    coordinatesx = rospy.Subscriber('coordinatesx', Float32, callback2)   
    circleRadius = rospy.Subscriber('circleRadius', Float32, callbackR)  
    

    while not rospy.is_shutdown():
        pubz.publish(msg)
        #rospy.loginfo(rospy.get_caller_id() + str(radius) + " starting angular %f", msg.angular.z)	 
    	
        rate.sleep()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



def callback2(data):
    pub=rospy.Publisher('/cmd_vel',Twist,queue_size=2)
    if(data.data > 270 and data.data < 330):
	msg.angular.z=0
        msg.linear.x=0.3
    elif(data.data > 330):
        msg.angular.z=-0.2
        msg.linear.x=0.0
    elif(data.data < 270):
        msg.angular.z = 0.2
        msg.linear.x = 0.0
   
    rospy.loginfo(rospy.get_caller_id() + " I heard-x %f", data.data)
    pub.publish(msg)

def callbackR(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard-radius %f", data.data)
    pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    radius = data.data
    if(data.data > 320 ):
        coordinatesx.unregister()
	msg.angular.z=0.0
        msg.linear.x=0.0
        #pub.publish(msg)
        #execfile('navigation.py')
        os.system('python navigation.py')
        rospy.signal_shutdown('done goes to the object')
        


    

if __name__ == '__main__':
    listener()


