import rospy
import math
from mvsim_msgs import SrvSetPose_pb2
from mvsim_msgs import SrvSetControllerTwist_pb2
from mvsim_msgs import TimeStampedPose_pb2
from mvsim_comms import pymvsim_comms
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

client = pymvsim_comms.mvsim.Client()
bridge = CvBridge()
kernal = np.ones((5, 5), "uint8")
red_BGR = (0, 0, 255)
blue_BGR = (255, 0, 0)
player_lower = np.array([115, 0, 0], np.uint8)
player_upper = np.array([170, 255, 40], np.uint8)
ball_lower = np.array([126, 10, 30], np.uint8)
ball_upper = np.array([170, 255, 120], np.uint8)
vlineal = 0

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
                if texto == "Pelota":
                    return x
                else:
                    continue



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
    client.callService('set_controller_twist', req.SerializeToString())


def get_minvalue(inputlist):
 
    #get the minimum value in the list
    min_value = min(inputlist)
 
    #return the index of minimum value 
    min_index=inputlist.index(min_value)
    return min_index

class moverRobot(object):

    def __init__(self):
        self.robots = ["Rojo1", "Rojo2", "Rojo3", "Rojo4"]
        self.subMVSIM = client.subscribeTopic("/pelota/pose", self.onPoseMessage)  
        self.subMVSIM2 = client.subscribeTopic("/"+self.robots[0]+"/pose", self.onPoseMessage1)
        self.subMVSIM3 = client.subscribeTopic("/"+self.robots[1]+"/pose", self.onPoseMessage2)
        self.subMVSIM4 = client.subscribeTopic("/"+self.robots[2]+"/pose", self.onPoseMessage3)   
        self.subMVSIM5 = client.subscribeTopic("/"+self.robots[3]+"/pose", self.onPoseMessage4)     

    def onPoseMessage1(self, msgType, msg):
        assert(msgType == "mvsim_msgs.TimeStampedPose")
        p = TimeStampedPose_pb2.TimeStampedPose()
        p.ParseFromString(bytes(msg))
        self.theta_robot1 = (p.pose.yaw +math.pi) % (math.pi*2)
        self.x_robot1 = p.pose.x
        self.y_robot1 = p.pose.y

    def onPoseMessage2(self, msgType, msg):
        assert(msgType == "mvsim_msgs.TimeStampedPose")
        p = TimeStampedPose_pb2.TimeStampedPose()
        p.ParseFromString(bytes(msg))
        self.theta_robot2 = (p.pose.yaw +math.pi) % (math.pi*2)
        self.x_robot2 = p.pose.x
        self.y_robot2 = p.pose.y

    def onPoseMessage3(self, msgType, msg):
        assert(msgType == "mvsim_msgs.TimeStampedPose")
        p = TimeStampedPose_pb2.TimeStampedPose()
        p.ParseFromString(bytes(msg))
        self.theta_robot3 = (p.pose.yaw +math.pi) % (math.pi*2)
        self.x_robot3 = p.pose.x
        self.y_robot3 = p.pose.y

    def onPoseMessage4(self, msgType, msg):
        assert(msgType == "mvsim_msgs.TimeStampedPose")
        p = TimeStampedPose_pb2.TimeStampedPose()
        p.ParseFromString(bytes(msg))
        self.theta_robot4 = (p.pose.yaw +math.pi) % (math.pi*2)
        self.x_robot4 = p.pose.x
        self.y_robot4 = p.pose.y




    def onPoseMessage(self, msgType, msg):
        assert(msgType == "mvsim_msgs.TimeStampedPose")
        p = TimeStampedPose_pb2.TimeStampedPose()
        p.ParseFromString(bytes(msg))
        #print("[pose callback] received: pose=\n" + str(p))
        self.y_pelota = p.pose.y
        self.x_pelota = p.pose.x
        #pasar a coordenadas gaussianas
        
        x1 = self.x_pelota - self.x_robot1
        y1 = self.y_pelota - self.y_robot1
        r1 = math.sqrt(math.pow(y1,2)+math.pow(x1,2))

        x2 = self.x_pelota - self.x_robot2
        y2 = self.y_pelota - self.y_robot2
        r2 = math.sqrt(math.pow(y2,2)+math.pow(x2,2))

        x3 = self.x_pelota - self.x_robot3
        y3 = self.y_pelota - self.y_robot3
        r3 = math.sqrt(math.pow(y3,2)+math.pow(x3,2))

        x4 = self.x_pelota - self.x_robot4
        y4 = self.y_pelota - self.y_robot4
        r4 = math.sqrt(math.pow(y4,2)+math.pow(x4,2))

        distancias = [r1, r2, r3, r4]
        listay = [y1, y2, y3, y4]
        listax = [x1, x2, x3, x4]
        index = get_minvalue(distancias)

        if index == 0:
            theta_rad =  (math.atan(y1/x1))
            theta_robot = self.theta_robot1
        elif index == 1:
            theta_rad =  (math.atan(y2/x2))
            theta_robot = self.theta_robot2
        elif index == 2:
            theta_rad =  (math.atan(y3/x3))
            theta_robot = self.theta_robot3
        else:
            theta_rad =  (math.atan(y4/x4))
            theta_robot = self.theta_robot4

        
        
        if listax[index] > 0:
            if listay[index] > 0:
                print("Primer Cuadrante")
            if  listay[index] < 0:
                print("Cuarto Cuadrante")
                theta_rad = 2*math.pi + theta_rad
        else:
            if listay[index] > 0:
                print("Segundo Cuadrante")
                theta_rad = math.pi + theta_rad
            if listay[index] < 0:
                theta_rad = math.pi + theta_rad
                print("Tercer cuadrante")
        

        for robot in self.robots:
            if robot != self.robots[index]:
                sendRobotTwistSetpoint(client, robot, 0, 0, 0)
            else:
                pass
        
        beta = (theta_robot - theta_rad)

        if   -0.05 < beta < 0.05:
            print("Mover hacia adelante")
            sendRobotTwistSetpoint(client, self.robots[index], -vlineal, 0, 0)
        elif beta > 0.05:
            if beta < math.pi:
                sendRobotTwistSetpoint(client, self.robots[index], 0, 0, -0.6)
                print("girar a derecha")
            else:
                sendRobotTwistSetpoint(client, self.robots[index], 0, 0, 0.6)
                print("girar a izq")
        else:
            if beta > -math.pi:
                sendRobotTwistSetpoint(client, self.robots[index], 0, 0, 0.6)
                print("girar a izq")
            else:
                sendRobotTwistSetpoint(client, self.robots[index], 0, 0, -0.6)
                print("girar a dch")
        
    
        print("objetivo es: " + str(theta_rad))

        if abs(self.x_pelota) > 11.13 or abs(self.y_pelota) > 7.13:
            self.restartMatch()
            print("Match Restarted, ball left playing area")
        else:
            #print("match goes on")
            pass



    def setObjectPose(self, client, objectName, x, y, theta_rad):
        # Send a set pose request:
        req = SrvSetPose_pb2.SrvSetPose()
        req.objectId = objectName  # vehicle/robot/object name in MVSIM
        req.pose.x = x
        req.pose.y = y
        req.pose.z = 0
        req.pose.yaw = theta_rad
        req.pose.pitch = 0
        req.pose.roll = 0  # math.radians(0.0)
        # ret =
        client.callService('set_pose', req.SerializeToString())
    
    def restartMatch(self):
        self.setObjectPose(client, "Rojo1", 2, 2, 0)
        self.setObjectPose(client, "Rojo2", 2, -2, 0)
        self.setObjectPose(client, "Rojo3", 6, 2, 0)
        self.setObjectPose(client, "Rojo4", 6, -2, 0)
        self.setObjectPose(client, "PRojo", 10.3, 0, math.radians(90))
        self.setObjectPose(client, "Azul1", -2, 2, 0)
        self.setObjectPose(client, "Azul2", -2, -2, 0)
        self.setObjectPose(client, "Azul3", -6, 2, 0)
        self.setObjectPose(client, "Azul4", -6, -2, 0)
        self.setObjectPose(client, "PAzul", -10.3, 0, math.radians(270))
        self.setObjectPose(client, "block003", 0, 0, 0)
        Lista = ["Rojo1", "Rojo2", "Rojo3", "Rojo4",
              "Azul1", "Azul2", "Azul3", "Azul4",
               "PRojo", "PAzul" ]
        for veh in Lista:
            sendRobotTwistSetpoint(client, veh, 0, 0, 0)
        




        


class moverPorteroRojo(object):
    def __init__(self):
        rospy.Subscriber("PRojo/camera1", Image, self.image_callback)
        client.subscribeTopic("/PRojo/pose", self.onPoseMessage)

    def image_callback(self, msg):
        #print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            detectarColor(cv2_img, player_lower, player_upper, "Player", blue_BGR, blue_BGR)
            x_pelota = detectarColor(cv2_img, ball_lower, ball_upper, "Pelota", red_BGR, red_BGR)
            if x_pelota is not None:
                if x_pelota > 340 and self.y_robot < 0.7:
                    #print("Mover hacia la derecha")
                    sendRobotTwistSetpoint(client, "PRojo", 0.3, 0, 0)
                elif x_pelota <300 and self.y_robot > -0.7:
                    #print("Mover a la izquierda")
                    sendRobotTwistSetpoint(client, "PRojo", -0.3, 0, 0)
                else:
                    #print("Parar robot")
                    sendRobotTwistSetpoint(client, "PRojo", 0, 0, 0)
            else:
                sendRobotTwistSetpoint(client, "PRojo", 0, 0, 0)
                #print("Pelota no visible, Parar robot")
            cv2.imshow('PRojo-Deteccion de colores', cv2_img)
            cv2.imwrite('/PRojo/camera1.jpeg', cv2_img)
            cv2.waitKey(50)  

    def onPoseMessage(self, msgType, msg):
        assert(msgType == "mvsim_msgs.TimeStampedPose")
        p = TimeStampedPose_pb2.TimeStampedPose()
        p.ParseFromString(bytes(msg))
        #print("[pose callback] received: pose=\n" + str(p))
        self.y_robot = p.pose.y





if __name__ == "__main__":
    client.setName("Equipo_rojo")
    print("Connecting to server...")
    client.connect()
    print("Connected successfully.")
    rospy.init_node("Equipo_Rojo", anonymous= True)
    print("Indicate speed of robots (recommended 0.4-0.6):")

    while True:
        vlineal = input()
        try:
            float(vlineal)
        except ValueError:
            print("Please introduce a number")
        else:
            vlineal = float(vlineal)
            break

    Start = moverRobot()
    PRojo = moverPorteroRojo()
    rospy.spin()


