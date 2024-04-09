#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist



movement = {
    "palante": [0.1,0],
    "right": [ 0, -0.1],
    "left": [0,0.1]
}


def turn_right():
    acc = 0
    #rospy.init_node('turn_right_node', anonymous=True)
    #set move
    move = Twist()
    #move.linear.x = 0.1
    move.angular.z = -0.1
    #publish
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(1)
    pub.publish(move)
    while acc<2:
        pub.publish(move)
        rate.sleep()
        acc +=1
    




def turn_left():
    acc = 0
    #rospy.init_node('turn_right_node', anonymous=True)
    #set move
    move = Twist()
    #move.linear.x = 0.1
    move.angular.z = 0.3
    #publish
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(1)
    pub.publish(move)
    while acc<2:
        pub.publish(move)
        rate.sleep()
        acc +=1




def palante():
    acc = 0
    #rospy.init_node('turn_right_node', anonymous=True)
    #set move
    move = Twist()
    move.linear.x = 0.1
    #publish
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(1)
    
    while acc<2:
        pub.publish(move)
        rate.sleep()
        acc +=1

#palante()