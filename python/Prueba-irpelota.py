from mvsim_comms import pymvsim_comms
from mvsim_msgs import TimeStampedPose_pb2
from mvsim_msgs import SrvSetPose_pb2
from mvsim_msgs import SrvSetControllerTwist_pb2
import time
import math


client = pymvsim_comms.mvsim.Client()


def onPosePelota(msgType, msg):
    assert(msgType == "mvsim_msgs.TimeStampedPose")
    p = TimeStampedPose_pb2.TimeStampedPose()
    p.ParseFromString(bytes(msg))
    #print("[pose callback] received: pose=\n" + str(p))
    global y_pelota, x_pelota
    x_pelota, y_pelota = p.pose.x, p.pose.y

def onPoseRobot(msgType, msg):
    assert(msgType == "mvsim_msgs.TimeStampedPose")
    p = TimeStampedPose_pb2.TimeStampedPose()
    p.ParseFromString(bytes(msg))
    #print("[pose callback] received: pose=\n" + str(p))
    global y_robot, x_robot, robot_yaw
    x_robot, y_robot, robot_yaw = p.pose.x, p.pose.y, p.pose.yaw

def robotPelotaAlineados():
    x = x_pelota - x_robot #convertimos el robot en centro de coordenadas, la pelota esta en coiordenadas (x,y)
    y = y_pelota - y_robot
    #pasamos a coordenadas gaussianas
    theta_rad = math.atan(y/x)
    if robot_yaw > (theta_rad - 0.05) or robot_yaw <(theta_rad + 0.05):
        return "Adelante"
    elif robot_yaw < 0:
        return "Izquierda"
    else:
        return "Derecha"


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



if __name__ == "__main__":
    client.setName("tutorial1")
    print("Connecting to server...")
    client.connect()
    print("Connected successfully.")
    
    while True:
        client.subscribeTopic("/pelota/pose", onPosePelota) 
        client.subscribeTopic("/robot/pose", onPoseRobot) 
        alineados = robotPelotaAlineados()
        if alineados is "Adelante":
            sendRobotTwistSetpoint(client, "Robot", 0.2, 0, 0)
        elif alineados is "Izquierda": 
            sendRobotTwistSetpoint(client, "Robot", 0, 0, -0.1)
        else:
            sendRobotTwistSetpoint(client, "Robot", 0, 0, 0.1)


