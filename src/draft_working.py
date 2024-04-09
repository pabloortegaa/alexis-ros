#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from utils import turn_left, turn_right, palante



class LoadImage(object):

    def __init__(self):
    
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()

    def camera_callback(self,data):
            try:
                # We select bgr8 because its the OpenCV encoding by default
                image = self.bridge_object.imgmsg_to_cv2(data)
                        # Convert image to HSV color space
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                
                # Define lower and upper bounds for white color
                lower_white = np.array([0, 0, 150])
                upper_white = np.array([180, 50, 255])
                
                # Threshold the HSV image to get only white colors
                mask = cv2.inRange(hsv, lower_white, upper_white)
                #apply the mask to the image
                image = cv2.bitwise_and(image, image, mask=mask)
                # sensor 1
                # define the coordinates of the center of the image
                center_x = image.shape[1] // 2 
                center_y = image.shape[0] // 2 + 220
                # define the size of the sensor
                sensor_size = 75
                # draw the sensor
                #cv2.rectangle(image, (center_x - sensor_size, center_y - sensor_size), (center_x + sensor_size, center_y + sensor_size), (255, 255, 255), 2)#white
                #show the center of the rectangle
                cv2.circle(image, (center_x, center_y), 2, (255, 255, 255), 2)
                #draw an horizontal line to show the direction of the sensor


                #draw a point 

                #cv2.circle(image, (center_x, center_y), 2, (255, 255, 0), 2)
                #count the number of white pixels in the left and right side of the sensor
                #left
                left = 0
                for i in range(sensor_size):
                    #cv2.circle(image, (center_x - i, center_y), 2, (0, 5, 255), 2)
                    #print(image[center_y, center_x-i])
                    if image[center_y, center_x-i][0] != 0 or image[center_y, center_x-i][1] != 0 or image[center_y, center_x-i][2] != 0:

                        left += 1

                right = 0
                for i in range(sensor_size):
                    #cv2.circle(image, (center_x + i, center_y), 2, (0, 5, 255), 2)
                    #print(image[center_y, center_x+i])
                    if image[center_y, center_x+i][0] != 0 or image[center_y, center_x+i][1] != 0 or image[center_y, center_x+i][2] != 0:
                        right += 1
                print("-------------------")
                print("left:", left)
                print("right:", right)
                


                sum_left_right = left + right

                #si sum_left_right es par
                #if sum_left_right % 2 == 0:
                if left > right:
                    if left - right > 10:

                        print("turn left")
                        if left == sensor_size:
                            # we detect a corner
                            print("corner detected")
                            print("palante")
                            print("turn left")
                    else:
                        print("palante")
                    #turn left

                elif right > left:
                    if right - left > 10:
                        print("turn right")
                        
                        if right == sensor_size:
                            # detect corner
                            print("corner detected")
                            print("palante")
                            print("turn right")
                    else:
                        print("palante")
                
                else: 
                    print("palante")

                cv_image = image
                cv2.imshow("suuuuu",cv_image)
                print("---------------")


            


        
            except CvBridgeError as e:
                print(e)
            example_path = '/home/husarion/husarion_ws/src/unit2/test_image_1.jpg'    
            img = cv2.imread(example_path)
            drone_image = cv2.imwrite('test_image_1.jpg',cv_image)
            #cv2.imshow('image',img)
            #save the image
            
            #cv2.waitKey(0)
            #cv2.destroyAllWindows()
        



def main():
    load_image_object = LoadImage()
    rospy.init_node('load_image_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()