from mvsim_comms import pymvsim_comms
from mvsim_msgs import TimeStampedPose_pb2
from mvsim_msgs import ObservationLidar2D_pb2
import time


# Callback for subscribed topic:
def onPoseMessage(msgType, msg):
    assert(msgType == "mvsim_msgs.TimeStampedPose")
    p = TimeStampedPose_pb2.TimeStampedPose()
    p.ParseFromString(bytes(msg))
    print("Y pose = "+  str(p.pose.y))
    #print("[pose callback] received: pose=\n" + str(p))




if __name__ == "__main__":
    client = pymvsim_comms.mvsim.Client()
    client.setName("tutorial1")
    print("Connecting to server...")
    client.connect()
    print("Connected successfully.")

    # Subscribe to "/r1/pose"
    
    
    client.subscribeTopic("/PRojo/pose", onPoseMessage)


    time.sleep(200.0)