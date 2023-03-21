import rospy
import math
import time
from mvsim_msgs import SrvSetPose_pb2
from mvsim_msgs import SrvSetControllerTwist_pb2
from mvsim_msgs import TimeStampedPose_pb2
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

def get_minvalue(inputlist):
 
    #get the minimum value in the list
    min_value = min(inputlist)
 
    #return the index of minimum value 
    min_index=inputlist.index(min_value)
    return min_index

class moverRobot(object):

    def __init__(self):
        self.robots = ["Azul1", "Azul2", "Azul3", "Azul4"]
        self.subMVSIM = client.subscribeTopic("/pelota/pose", self.onPoseMessage)  
        self.subMVSIM2 = client.subscribeTopic("/"+self.robots[0]+"/pose", self.onPoseMessage1)
        self.subMVSIM3 = client.subscribeTopic("/"+self.robots[1]+"/pose", self.onPoseMessage2)
        self.subMVSIM4 = client.subscribeTopic("/"+self.robots[2]+"/pose", self.onPoseMessage3)   
        self.subMVSIM5 = client.subscribeTopic("/"+self.robots[3]+"/pose", self.onPoseMessage4)     

    def onPoseMessage1(self, msgType, msg):
        assert(msgType == "mvsim_msgs.TimeStampedPose")
        p = TimeStampedPose_pb2.TimeStampedPose()
        p.ParseFromString(bytes(msg))
        self.theta_robot1 = (math.pi*2 - abs(p.pose.yaw)) % (math.pi*2)
        self.x_robot1 = p.pose.x
        self.y_robot1 = p.pose.y

    def onPoseMessage2(self, msgType, msg):
        assert(msgType == "mvsim_msgs.TimeStampedPose")
        p = TimeStampedPose_pb2.TimeStampedPose()
        p.ParseFromString(bytes(msg))
        self.theta_robot2 = (math.pi*2 - abs(p.pose.yaw)) % (math.pi*2)
        self.x_robot2 = p.pose.x
        self.y_robot2 = p.pose.y

    def onPoseMessage3(self, msgType, msg):
        assert(msgType == "mvsim_msgs.TimeStampedPose")
        p = TimeStampedPose_pb2.TimeStampedPose()
        p.ParseFromString(bytes(msg))
        self.theta_robot3 = (math.pi*2 - abs(p.pose.yaw)) % (math.pi*2)
        self.x_robot3 = p.pose.x
        self.y_robot3 = p.pose.y

    def onPoseMessage4(self, msgType, msg):
        assert(msgType == "mvsim_msgs.TimeStampedPose")
        p = TimeStampedPose_pb2.TimeStampedPose()
        p.ParseFromString(bytes(msg))
        self.theta_robot4 = (math.pi*2 - abs(p.pose.yaw)) % (math.pi*2)
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
        beta = theta_rad + 0.05
        alfa = theta_rad - 0.05
        for robot in self.robots:
            if robot != self.robots[index]:
                sendRobotTwistSetpoint(client, robot, 0, 0, 0)
            else:
                pass
        if beta > theta_robot > alfa:
            print("Mover hacia adelante")
            sendRobotTwistSetpoint(client, self.robots[index], 0.5, 0, 0)
        elif theta_robot < beta:
            print("girar a izquierda")
            sendRobotTwistSetpoint(client, self.robots[index], 0, 0, 0.6)
        else:
            sendRobotTwistSetpoint(client, self.robots[index], 0, 0, -0.6)
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


