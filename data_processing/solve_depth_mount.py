import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

# World frame relative to depth camera
depth_tf = np.load("configs/tf_camera.npz")
depth_R = Rotation.from_matrix(depth_tf["R"])
depth_t = depth_tf["t"]

# Headset pose relative to world frame
quest_tf = np.load("configs/calibration.npz")
quest_R = Rotation.from_quat(quest_tf["rel_rot"])
quest_t = quest_tf["rel_pos"]

# Solve relative transform from headset to depth camera
depth_R_inv = depth_R.inv()
rel_R = depth_R_inv * quest_R
rel_t = depth_R_inv.apply(quest_t - depth_t)

# Visualize relative transform
world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
depth_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
depth_frame.rotate(rel_R.as_matrix())
depth_frame.translate(rel_t)
o3d.visualization.draw_geometries([world_frame, depth_frame])
np.savez("configs/head2depth.npz", orn=rel_R.as_quat(), pos=rel_t)