from mvsim_comms import pymvsim_comms
from mvsim_msgs import TimeStampedPose_pb2
from mvsim_msgs import SrvSetPose_pb2
from mvsim_msgs import SrvSetControllerTwist_pb2
import time
import math

client = pymvsim_comms.mvsim.Client()
y_pelota, x_pelota = 0.0, 0.0

def setObjectPose(client, objectName, x, y, theta_rad):
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



def onPoseMessage(msgType, msg):
    assert(msgType == "mvsim_msgs.TimeStampedPose")
    p = TimeStampedPose_pb2.TimeStampedPose()
    p.ParseFromString(bytes(msg))
    #print("[pose callback] received: pose=\n" + str(p))
    global y_pelota
    global  x_pelota
    x_pelota = p.pose.x
    y_pelota = p.pose.y
    if abs(x_pelota) > 11.13 or abs(y_pelota) > 7.13:
        restartMatch()
        print("Match Restarted, ball left playing area")
    else:
        print("match goes on")

def restartMatch():
    setObjectPose(client, "Rojo1", 2, 2, math.radians(180))
    setObjectPose(client, "Rojo2", 2, -2, math.radians(180))
    setObjectPose(client, "Rojo3", 6, 2, math.radians(180))
    setObjectPose(client, "Rojo4", 6, -2, math.radians(180))
    setObjectPose(client, "PRojo", 10.3, 0, math.radians(90))
    setObjectPose(client, "Azul1", -2, 2, 0)
    setObjectPose(client, "Azul2", -2, -2, 0)
    setObjectPose(client, "Azul3", -6, 2, 0)
    setObjectPose(client, "Azul4", -6, -2, 0)
    setObjectPose(client, "PAzul", -10.3, 0, math.radians(270))
    setObjectPose(client, "block003", 0, 0, 0)
    Lista = ["Rojo1", "Rojo2", "Rojo3", "Rojo4",
              "Azul1", "Azul2", "Azul3", "Azul4",
               "PRojo", "PAzul" ]
    for veh in Lista:
        sendRobotTwistSetpoint(client, veh, 0, 0, 0)



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
        client.subscribeTopic("/pelota/pose", onPoseMessage) 
        time.sleep(100)

        

