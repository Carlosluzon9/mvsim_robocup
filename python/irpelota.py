import rospy
import math
import time
from mvsim_msgs import SrvSetPose_pb2
from mvsim_msgs import SrvSetControllerTwist_pb2
from mvsim_msgs import TimeStampedPose_pb2
from nav_msgs.msg import Odometry
from mvsim_comms import pymvsim_comms

client = pymvsim_comms.mvsim.Client()

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


class moverRobot(object):

    def __init__(self):
        #self.subROS = rospy.Subscriber("Rojo1/odom", Odometry, self.callback)
        self.subMVSIM = client.subscribeTopic("/pelota/pose", self.onPoseMessage)  
        self.subMVSIM2 = client.subscribeTopic("/Rojo1/pose", self.onPoseMessage1)
    
    def callback(self, msg):
        self.x_robot = msg.pose.pose.position.x
        self.y_robot = msg.pose.pose.position.y
        self.theta_robot = - ((msg.pose.pose.orientation.z * math.pi)/2) 

    def onPoseMessage1(self, msgType, msg):
        assert(msgType == "mvsim_msgs.TimeStampedPose")
        p = TimeStampedPose_pb2.TimeStampedPose()
        p.ParseFromString(bytes(msg))
        #print("[pose callback] received: pose=\n" + str(p))
        self.theta_robot = p.pose.yaw +math.pi
        self.x_robot = p.pose.x
        self.y_robot = p.pose.y




    def onPoseMessage(self, msgType, msg):
        assert(msgType == "mvsim_msgs.TimeStampedPose")
        p = TimeStampedPose_pb2.TimeStampedPose()
        p.ParseFromString(bytes(msg))
        #print("[pose callback] received: pose=\n" + str(p))
        self.y_pelota = p.pose.y
        self.x_pelota = p.pose.x
        #pasar a coordenadas gaussianas
        x = self.x_pelota - self.x_robot 
        y = self.y_pelota - self.y_robot
        theta_rad =  (math.atan(y/x))

        print("giro robot es: " + str(self.theta_robot))
        
        if x > 0:
            if y > 0:
                print("Primer Cuadrante")
            if y<0:
                print("Cuarto Cuadrante")
                theta_rad = 2*math.pi + theta_rad
        else:
            if y > 0:
                print("Segundo Cuadrante")
                theta_rad = math.pi + theta_rad
            if y < 0:
                theta_rad = math.pi + theta_rad
                print("Tercer cuadrante")
        beta = theta_rad + 0.05
        alfa = theta_rad - 0.05
        if beta > self.theta_robot > alfa:
            print("Mover hacia adelante")
            sendRobotTwistSetpoint(client, "Rojo1", -0.5, 0, 0)
        elif self.theta_robot < beta:
            print("girar a izquierda")
            sendRobotTwistSetpoint(client, "Rojo1", 0, 0, -0.2)
        else:
            sendRobotTwistSetpoint(client, "Rojo1", 0, 0, 0.2)
            print("girar a derecha")
    
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
        


if __name__ == "__main__":
    client.setName("tutorial1")
    print("Connecting to server...")
    client.connect()
    print("Connected successfully.")
    rospy.init_node("tutorial_1", anonymous= True)

    mover = moverRobot()
    rospy.spin()


