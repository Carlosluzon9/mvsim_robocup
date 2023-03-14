from mvsim_comms import pymvsim_comms
from mvsim_msgs import SrvGetPose_pb2, SrvGetPoseAnswer_pb2


client = pymvsim_comms.mvsim.Client()

def getRobotPose(client, robotName):
    req = SrvGetPose_pb2.SrvGetPose()
    req.objectId = robotName  # vehicle/robot/object name in MVSIM
    ret = client.callService('get_pose', req.SerializeToString())
    ans = SrvGetPoseAnswer_pb2.SrvGetPoseAnswer()
    ans.ParseFromString(ret)
    print(ans)

if __name__ == '__main__':
    client.setName("get-pose")
    client.connect()
    print("connected succesfully")

    getRobotPose(client, "Azul1")