from realsense_module import DepthCameraModule
from quest_robot_module import QuestRobotModule
import open3d as o3d
import numpy as np
from ip_config import *


class QuestCalibrationRobot(QuestRobotModule):
    def __init__(self, vr_ip, local_ip, pose_cmd_port):
        super().__init__(vr_ip, local_ip, pose_cmd_port)
        self.world_frame = None
        self.rel_pos = None
        self.rel_rot = None

    # World frame marks beginning of a program.
    def receive(self):
        data, _ = self.wrist_listener_s.recvfrom(1024)
        data_string = data.decode()
        if data_string.startswith("WorldFrame"):
            data_string = data_string[11:]
            data_string = data_string.split(",")
            data_list = [float(data) for data in data_string]
            world_frame = np.array(data_list)
            self.world_frame = world_frame
            print("WorldFrame received.")
        elif data_string.startswith("Start"): # Should send a head pose to the server
            data_string = data_string[6:].split(",")
            data_list = [float(data) for data in data_string]
            head_tf = np.array(data_list)
            self.rel_pos, self.rel_rot = self.compute_rel_transform(head_tf) # In world frame
            print("Head pose recorded.")
        
    def save_calibration(self, filename):
        np.savez(f"configs/{filename}.npz", rel_pos=self.rel_pos, rel_rot=self.rel_rot)

    def is_ready(self):
        return self.world_frame is not None and self.rel_pos is not None and self.rel_rot is not None

depth_camera = DepthCameraModule(visualize=False)
quest_robot = QuestCalibrationRobot(VR_HOST, LOCAL_HOST, POSE_CMD_PORT)

while True:
    quest_robot.receive()
    if quest_robot.is_ready():
        quest_robot.save_calibration(f"calibration")
        break

while True:
    pcd = depth_camera.receive()
    o3d_pcd = o3d.geometry.PointCloud()
    o3d_pcd.points = o3d.utility.Vector3dVector(pcd[:,:3])
    o3d_pcd.colors = o3d.utility.Vector3dVector(pcd[:,3:])
    o3d.visualization.draw_geometries_with_vertex_selection([o3d_pcd])
    o3d.io.write_point_cloud("configs/raw_point_cloud.ply", o3d_pcd)

