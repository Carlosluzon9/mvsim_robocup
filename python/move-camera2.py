import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
from mvsim_comms import pymvsim_comms
from mvsim_msgs import SrvGetPose_pb2, SrvGetPoseAnswer_pb2
from mvsim_msgs import SrvSetControllerTwist_pb2
import cv2
import time
import numpy as np

# Instantiate CvBridge
bridge = CvBridge()
kernal = np.ones((5, 5), "uint8")
client = pymvsim_comms.mvsim.Client()

y_robot = 1

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        red_BGR = (0, 0, 255)
        blue_BGR = (255, 0, 0)
        player_lower = np.array([115, 0, 0], np.uint8)
        player_upper = np.array([170, 255, 40], np.uint8)
        ball_lower = np.array([126, 10, 30], np.uint8)
        ball_upper = np.array([170, 255, 120], np.uint8)
        detectarColor(cv2_img, player_lower, player_upper, "Player", blue_BGR, blue_BGR)
        x_pelota = detectarColor(cv2_img, ball_lower, ball_upper, "Pelota", red_BGR, red_BGR)
        getRobotPose(client, "PRojo")
        if x_pelota is not None:
            if x_pelota > 340 and y_robot < 0.7:
                print("Mover hacia la derecha")
                sendRobotTwistSetpoint(client, "PRojo", 0.2, 0, 0)
            elif x_pelota <300 and y_robot > -0.7:
                print("Mover a la izquierda")
                sendRobotTwistSetpoint(client, "PRojo", -0.2, 0, 0)
            else:
                print("Parar robot")
                sendRobotTwistSetpoint(client, "PRojo", 0, 0, 0)
        else:
            sendRobotTwistSetpoint(client, "PRojo", 0, 0, 0)
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
            if(area > 50):
                x, y, w, h = cv2.boundingRect(contour)
                imagencv2 = cv2.rectangle(imagencv2, (x, y),
                                       (x + w, y + h),
                                       color_rectangulo, 2)
              
                cv2.putText(imagencv2, texto, (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, color_letras)
                return x



def sendRobotTwistSetpoint(client, robotName, vx, vy, w):
    # (vx,vy) in local coordinates [m/s]
    # (w) in [rad/s]

    req = SrvSetControllerTwist_pb2.SrvSetControllerTwist()
    req.objectId = robotName  # vehicle/robot/object name in MVSIM
    req.twistSetPoint.vx = vx
    req.twistSetPoint.vy = vy
    req.twistSetPoint.vz = 0
    req.twistSetPoint.wx = 0
    req.twistSetPoint.wy = 0
    req.twistSetPoint.wz = w
    # ret =
    client.callService('set_controller_twist', req.SerializeToString())

def getRobotPose(client, robotName):
    req = SrvGetPose_pb2.SrvGetPose()
    req.objectId = robotName  # vehicle/robot/object name in MVSIM
    ret = client.callService('get_pose', req.SerializeToString())
    ans = SrvGetPoseAnswer_pb2.SrvGetPoseAnswer()
    ans.ParseFromString(ret)
    print(ans)




def main():
    rospy.init_node('Portero1')
    client.setName("Portero")
    print("Connecting to  mvsim server...")
    client.connect()
    print("Connected successfully.")

    # Define your image topic
    image_topic = "/PRojo/camera1"
    # Set up your subscriber and define its callback

    while not rospy.is_shutdown():
        #client.subscribeTopic("/PRojo/pose", onPoseMessage)
        rospy.Subscriber(image_topic, Image, image_callback)  

        time.sleep(50)
    #rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c


if __name__ == '__main__':
    main()