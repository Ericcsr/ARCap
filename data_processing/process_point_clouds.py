import os
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
from argparse import ArgumentParser
import time

def compute_world_frame_pcd(head_pos, head_orn, head2depth_pos, head2depth_orn, pcd):
    # pcd is collected in depth camera frame
    head_orn = Rotation.from_quat(head_orn)
    head2depth_orn = Rotation.from_quat(head2depth_orn)
    pcd_pos = pcd[:,:3]
    pcd_color = pcd[:,3:]
    # Convert pcd to world frame, first to head frame then to world frame
    pcd_pos_head = head2depth_orn.inv().apply(pcd_pos - head2depth_pos)
    pcd_pos_world = head_orn.apply(pcd_pos_head) + head_pos
    return np.hstack((pcd_pos_world, pcd_color))

def to_world(pcd, head_pos, head_orn):
    pcd_pos = pcd[:,:3]
    pcd_color = pcd[:,3:]
    pcd_world = head_orn.apply(delta_orn.apply(pcd_pos) + delta_pos) + head_pos
    return np.hstack((pcd_world, pcd_color))

def crop_pcd(pcd, min_bound, max_bound):
    pcd_pos = pcd[:,:3]
    pcd_color = pcd[:,3:]
    mask = np.logical_and(np.all(pcd_pos >= min_bound, axis=1), np.all(pcd_pos <= max_bound, axis=1))
    pcd_pos = pcd_pos[mask]
    pcd_color = pcd_color[mask]
    return np.hstack((pcd_pos, pcd_color))

# Same data processing should be used before training.
def read_data(path):
    files = os.listdir(path)
    # Sort files by time stamp right_data_{time.time()}.npz
    files.sort(key=lambda x: float(x.split("_")[2].split(".")[0]))
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    first = True
    for file in files:
        data = np.load(f"{path}/{file}")
        pcd = data["point_cloud"]
        head_pos, head_orn = data["head_pos"], data["head_orn"]
        #world_pcd = compute_world_frame_pcd(head_pos, head_orn, head2depth_pos, head2depth_orn, pcd)
        world_pcd = to_world(pcd, head_pos, Rotation.from_quat(head_orn))
        if first:
            vis_pcd = o3d.geometry.PointCloud()
            frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
            #world_pcd = crop_pcd(world_pcd, (-0.1,-0.1,-0.1), (0.4,0.4,0.4))
            world_pcd = crop_pcd(world_pcd, (-0.7,0.1, 0.02), (0.,0.6,0.4))
            vis_pcd.points = o3d.utility.Vector3dVector(world_pcd[:,:3])
            vis_pcd.colors = o3d.utility.Vector3dVector(world_pcd[:,3:])
            vis.add_geometry(vis_pcd)
            vis.add_geometry(frame)
            first = False
            input("press enter to continue...")
        else:
            #world_pcd = crop_pcd(world_pcd, (-0.1,-0.1,-0.1), (0.4,0.4,0.4))
            world_pcd = crop_pcd(world_pcd, (-0.7,0.1, 0.02), (0.,0.6,0.4))
            vis_pcd.points = o3d.utility.Vector3dVector(world_pcd[:,:3])
            vis_pcd.colors = o3d.utility.Vector3dVector(world_pcd[:,3:])
            vis.update_geometry(vis_pcd)
            vis.poll_events()
            vis.update_renderer()
    vis.destroy_window()

depth_tf = np.load("configs/tf_camera.npz")
depth_R = Rotation.from_matrix(depth_tf["R"])
depth_t = depth_tf["t"]
quest_tf = np.load("configs/calibration.npz")
quest_R = Rotation.from_quat(quest_tf["rel_rot"])
quest_t = quest_tf["rel_pos"]
delta_orn = quest_R.inv() * depth_R
delta_pos = quest_R.inv().apply(depth_t - quest_t)

parser = ArgumentParser()
parser.add_argument("--path", type=str, required=True)
args = parser.parse_args()

read_data(f"data/{args.path}")
