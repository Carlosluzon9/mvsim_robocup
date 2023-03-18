import rospy
import math
import time
from mvsim_msgs import SrvSetControllerTwist_pb2
from mvsim_msgs import TimeStampedPose_pb2
from nav_msgs.msg import Odometry
from mvsim_comms import pymvsim_comms

client = pymvsim_comms.mvsim.Client()


class moverRobot(object):

    def __init__(self):
        self.subROS = rospy.Subscriber("Rojo1/odom", Odometry, self.callback)
        self.subMVSIM = client.subscribeTopic("/pelota/pose", self.onPoseMessage)  

    def callback(self, msg):
        self.x_robot = msg.pose.pose.position.x
        self.y_robot = msg.pose.pose.position.y
        self.theta_robot = (msg.pose.pose.orientation.z * math.pi)/2
    
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
        theta_rad = math.atan(y/x)
        print("objetivo es: " + str(theta_rad))
        print("giro robot es: " + str(self.theta_robot))
        
        if (theta_rad + 0.05) > self.theta_robot > (theta_rad - 0.05):
           self.sendRobotTwistSetpoint(client, "Rojo1", 0.2, 0, 0)
        elif self.theta_robot < theta_rad:
           self.sendRobotTwistSetpoint(client, "Rojo1", 0, 0, 0.2)
        elif self.theta_robot > theta_rad:
           self.sendRobotTwistSetpoint(client, "Rojo1", 0, 0, -0.2)
        

    def sendRobotTwistSetpoint(self, client, robotName, vx, vy, w):
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
        


if __name__ == "__main__":
    client.setName("tutorial1")
    print("Connecting to server...")
    client.connect()
    print("Connected successfully.")
    rospy.init_node("tutorial_1", anonymous= True)

    mover = moverRobot()
    rospy.spin()


