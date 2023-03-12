import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import time
import numpy as np

# Instantiate CvBridge
bridge = CvBridge()

red_lower = np.array([136, 87, 111], np.uint8)
red_upper = np.array([180, 255, 255], np.uint8)
player_lower = np.array([125, 0, 0], np.uint8)
player_upper = np.array([179, 255, 70], np.uint8)
blue_lower = np.array([94, 80, 2], np.uint8)
blue_upper = np.array([120, 255, 255], np.uint8)

kernal = np.ones((5, 5), "uint8")

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        hsvFrame = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)
        
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
        player_mask = cv2.inRange(hsvFrame, player_lower, player_upper)
        blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

        red_mask = cv2.dilate(red_mask, kernal)
        res_red = cv2.bitwise_and(cv2_img, cv2_img, mask = red_mask)

        player_mask = cv2.dilate(player_mask, kernal)
        res_player = cv2.bitwise_and(cv2_img, cv2_img, mask = player_mask)

        blue_mask = cv2.dilate(blue_mask, kernal)
        res_blue = cv2.bitwise_and(cv2_img, cv2_img, mask = blue_mask)

        contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)      

        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > 300):
                x, y, w, h = cv2.boundingRect(contour)
                cv2_img = cv2.rectangle(cv2_img, (x, y), 
                                       (x + w, y + h), 
                                       (0, 0, 255), 2)
              
                cv2.putText(cv2_img, "Color Rojo", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 0, 255))    


        contours, hierarchy = cv2.findContours(player_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > 300):
                x, y, w, h = cv2.boundingRect(contour)
                cv2_img = cv2.rectangle(cv2_img, (x, y), 
                                       (x + w, y + h),
                                       (0, 255, 0), 2)
              
                cv2.putText(cv2_img, "Jugador", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        1.0, (0, 255, 0))   

        contours, hierarchy = cv2.findContours(blue_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
        

        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > 300):
                x, y, w, h = cv2.boundingRect(contour)
                cv2_img = cv2.rectangle(cv2_img, (x, y),
                                       (x + w, y + h),
                                       (255, 0, 0), 2)
              
                cv2.putText(cv2_img, "Blue Colour", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (255, 0, 0))


        cv2.imshow('PRojo-Deteccion de colores', cv2_img)
        cv2.imwrite('camera_image.jpeg', cv2_img)
        cv2.waitKey(50)

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/PRojo/camera1"
    # Set up your subscriber and define its callback
    while not rospy.is_shutdown():
        rospy.Subscriber(image_topic, Image, image_callback)


        
        time.sleep(50)
    #rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    

if __name__ == '__main__':
    main()