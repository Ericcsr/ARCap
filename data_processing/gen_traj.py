import numpy as np
import os
from scipy.spatial.transform import Rotation
from argparse import ArgumentParser
import open3d as o3d
right_hand_pos_offset = [0.05, -0.05, 0.1]
    
right_hand_orn_offset = Rotation.from_euler("xyz", [-np.pi, 0., 0.])

right_palm_orn_offset = np.array([-0.1, -0.05, 0.05, 0.0, 0.0, -np.pi/2])

o3d_vis = o3d.visualization.Visualizer()
o3d_vis.create_window(width=960, height=540)
initialized = [False]
pcd = o3d.geometry.PointCloud()
def process_wrist_pos(wrist_pos, wrist_orn):
    wrist_orn_ = wrist_orn * right_hand_orn_offset.inv()
    wrist_pos_ = wrist_orn_.apply(right_palm_orn_offset[:3]) + wrist_pos
    wrist_orn_ = wrist_orn_ * Rotation.from_euler("xyz",right_palm_orn_offset[3:])
    print(wrist_pos_)
    return wrist_pos_, wrist_orn_.as_quat()

def load_traj(folder_name):
    files = os.listdir(folder_name)
    files.sort(key=lambda x: float(x.split("_")[2][:-4]))
    wrist_poses = []
    wrist_orns = []
    hand_qs = []
    arm_qs = []
    for i,file in enumerate(files):
        data = np.load(f"{folder_name}/{file}")
        if "right_wrist_pos" in data.files and "left_wrist_pos" in data.files:
            handedness = "both"
        elif "right_wrist_pos" in data.files:
            handedness = "right"
        elif "left_wrist_pos" in data.files:
            handedness = "left"
        if handedness == "both":
            hand_q = data["hand_q"]
            arm_q = data["arm_q"]
        else:
            hand_q = data[f"{handedness}_hand_q"]
            arm_q = data[f"{handedness}_arm_q"]
        # if initialized[0] == False:
        #     pcd.points = o3d.utility.Vector3dVector(data["point_cloud"][:,:3])
        #     pcd.colors = o3d.utility.Vector3dVector(data["point_cloud"][:,3:])
        #     o3d_vis.add_geometry(pcd)
        #     initialized[0] = True
        # else:
        #     pcd.points = o3d.utility.Vector3dVector(data["point_cloud"][:,:3])
        #     pcd.colors = o3d.utility.Vector3dVector(data["point_cloud"][:,3:])
        #     o3d_vis.update_geometry(pcd)
        #     o3d_vis.poll_events()
        #     o3d_vis.update_renderer()
        hand_qs.append(hand_q)
        arm_qs.append(arm_q)
    return wrist_poses, wrist_orns, hand_qs, arm_qs

parser = ArgumentParser()
parser.add_argument("--root", type=str, required=True)
parser.add_argument("--file", type=str, default=None)
parser.add_argument("--exp_name", type=str, required=True)
args = parser.parse_args()

if args.file is None:
    files = os.listdir(f"data/{args.root}")
    files.remove("WorldFrame.npy")
    #files.remove("RobotFrame.npy")
    for i,file in enumerate(files):
        wrist_poses, wrist_orns, hand_qs, arm_qs = load_traj(f"data/{args.root}/{file}")
        np.savez(f"trajs/arcap_{args.exp_name}_{i}.npz", wrist_poses=wrist_poses, wrist_orns=wrist_orns, arm_qs = arm_qs, hand_qs=hand_qs)
else:
    wrist_poses, wrist_orns, hand_qs, arm_qs = load_traj(f"data/{args.root}/{args.file}")
    np.savez(f"trajs/arcap_{args.exp_name}.npz", wrist_poses=wrist_poses, wrist_orns=wrist_orns, arm_qs = arm_qs, hand_qs=hand_qs)