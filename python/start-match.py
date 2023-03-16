from mvsim_comms import pymvsim_comms
from mvsim_msgs import TimeStampedPose_pb2
from mvsim_msgs import SrvSetPose_pb2
import time
import math

client = pymvsim_comms.mvsim.Client()

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
    global y_pelota, x_pelota
    x_pelota, y_pelota = p.pose.x, p.pose.y

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
    setObjectPose(client, "pelota", 0, 0, 0)




if __name__ == "__main__":
    client.setName("tutorial1")
    print("Connecting to server...")
    client.connect()
    print("Connected successfully.")

    while True:
        client.subscribeTopic("/pelota/pose", onPoseMessage) 
        if abs(x_pelota) > 11.13 or abs(y_pelota) > 7.13:
            restartMatch()
            print("Match Restarted, ball left playing area")
        else:
            time.sleep(2)
            continue  

