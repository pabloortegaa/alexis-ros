#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Twist

global corner_var
corner_var = 0
global side_turn
side_turn = "center"
move = Twist()

def camera_callback(data):
    global corner_var
    global side_turn
    global move

    bridge_object = CvBridge()

    image = bridge_object.imgmsg_to_cv2(data)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0, 0, 150])
    upper_white = np.array([180, 50, 255])

    mask = cv2.inRange(hsv, lower_white, upper_white)
    image = cv2.bitwise_and(image, image, mask=mask)

    if side_turn != "center":
        sign_turn = 1 if side_turn == "left" else -1
        move.linear.x = 0.2
        move.angular.z = 1.4 * sign_turn
        corner_var -= 1
        if corner_var == 0:
            side_turn = "center"
    else:
        center_x = image.shape[1] // 2 
        center_y = image.shape[0] // 2 + 220
        sensor_size = 75

        left = sum(1 for i in range(sensor_size) if np.any(image[center_y, center_x-i]))
        right = sum(1 for i in range(sensor_size) if np.any(image[center_y, center_x+i]))

        if left > right:
            if left - right > 10:                        
                if left == sensor_size:
                    #CORNER DETECTED TO THE LEFT
                    move.linear.x = 0.3
                    corner_var = 6
                    side_turn = "left"
            else:
                #TURN LEFT
                move.linear.x = 0.1
                move.angular.z = 0.3
        elif right > left:
            if right - left > 10:
                if right == sensor_size:
                    #CORNER DETECTED TO THE RIGHT
                    move.linear.x = 0.3
                    corner_var = 6
                    side_turn = "right"
            else:
                #TURN RIGHT
                move.angular.z = - 0.3
        else: 
            move.linear.x = 0.1

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(2)
    pub.publish(move)
    print("corner_var", corner_var)
    print("---------------")

def main():
    rospy.init_node('load_image_node', anonymous=True)
    rospy.Subscriber('/camera_topic', Image, camera_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
