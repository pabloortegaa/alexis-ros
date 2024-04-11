#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Twist


global corner_var
global side_turn
global object_detected 
global sign_turn
global move


move = Twist()
class LineFollower(object):
    def __init__(self, corner_var, side_turn):
    
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image, self.camera_callback)
        #self.image_lidar = rospy.Subscriber("/scan",LaserScan, self.lidar_callback)
        self.bridge_object = CvBridge()
        #self.corner_var = corner_var
        #self.side_turn = side_turn
        
    def camera_callback(self,data):
        #bridge_object = CvBridge()

        image = bridge_object.imgmsg_to_cv2(data)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_white = np.array([0, 0, 150])
        upper_white = np.array([180, 50, 255])

        mask = cv2.inRange(hsv, lower_white, upper_white)
        image = cv2.bitwise_and(image, image, mask=mask)
        center_x = image.shape[1] // 2 
        center_y = image.shape[0] // 2 + 220
        sensor_size = 75
        left = sum(1 for i in range(sensor_size) if np.any(image[center_y, center_x-i]))
        right = sum(1 for i in range(sensor_size) if np.any(image[center_y, center_x+i]))

        if left > right:
            if left - right > 10:                        
                if left == sensor_size:
                    #CORNER DETECTED TO THE LEFT
                    #move.linear.x = 0.3
                    corner_var = 6
                    side_turn = "left"
            else:
                #TURN LEFT
                sign_turn = 1
        elif right > left:
            if right - left > 10:
                if right == sensor_size:
                    #CORNER DETECTED TO THE RIGHT
                    #move.linear.x = 0.3
                    corner_var = 6
                    side_turn = "right"
            else:
                #TURN RIGHT
                #move.angular.z = - 0.3
                sign_turn = -1
        else:
            #GO STRAIGHT  
            move.linear.x = 0.1
            sign_turn = 0
        

        example_path = 'test_image_1.jpg'    
        img = cv2.imread(example_path)
        drone_image = cv2.imwrite(example_path,image)


        #cv2.imshow('image',img)
        #save the image
        
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
    #def lidar_callback(self,data):
        #print('hello')
        # get the info of the lidar
        # if we detect and obstacle we stop the robot


sign_turn = 0
corner_var = 0 
side_turn = "center"
object_detected = False

if __name__ == '__main__':
   
    while True:
    
        
        rospy.init_node('load_image_node', anonymous=True)
        line_follower = LineFollower(corner_var, side_turn)
        #prueba = 0
        move = Twist()
        
        try:


            if object_detected:
                #go to that object
                #yolov8
                print("object detected")
            else:
                #follow the line   
                if side_turn != "center": # we are turning
                    sign_turn = 1 if side_turn == "left" else -1
                    move.linear.x = 0.2
                    move.angular.z = 1.4 * sign_turn
                    corner_var -= 1
                    if corner_var == 0:
                        side_turn = "center"
                else: # we are going straight
                    move.linear.x = 0.1
                    move.angular.z = 0.3 * sign_turn

            pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
            rate = rospy.Rate(2)
            pub.publish(move)
            print("corner_var", corner_var)
            print("---------------")
            #rate.sleep()


                    
            
        except KeyboardInterrupt:
            print("Shutting down")
            rospy.shutdown()
            cv2.destroyAllWindows()
            #break

#if __name__ == '__main__':
       # main()
    