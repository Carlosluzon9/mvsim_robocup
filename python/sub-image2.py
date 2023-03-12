import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import time

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        cv2.imshow('image', cv2_img)
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