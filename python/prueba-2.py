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
red_BGR = (0, 0, 255)
blue_BGR = (255, 0, 0)
player_lower = np.array([125, 0, 0], np.uint8)
player_upper = np.array([179, 255, 70], np.uint8)
ball_lower = np.array([0, 200, 120], np.uint8)
ball_upper = np.array([0, 255, 255], np.uint8)

kernal = np.ones((5, 5), "uint8")

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        detectarColor(cv2_img, player_lower, player_upper, "Player", blue_BGR, blue_BGR)
        detectarColor(cv2_img, ball_lower, ball_upper, "Pelota", red_BGR, red_BGR)
        cv2.imshow('PRojo-Deteccion de colores', cv2_img)
        cv2.imwrite('camera_image.jpeg', cv2_img)
        cv2.waitKey(50)



def detectarColor(imagencv2, HSV_lower, HSV_upper, texto, color_rectangulo, color_letras):
        
        hsvFrame = cv2.cvtColor(imagencv2, cv2.COLOR_BGR2HSV)
        color_mask = cv2.inRange(hsvFrame, HSV_lower, HSV_upper)
        color_mask = cv2.dilate(color_mask, kernal)
        contours, hierarchy = cv2.findContours(color_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)   
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > 100):
                x, y, w, h = cv2.boundingRect(contour)
                imagencv2 = cv2.rectangle(imagencv2, (x, y),
                                       (x + w, y + h),
                                       color_rectangulo, 2)
              
                cv2.putText(imagencv2, texto, (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, color_letras)





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