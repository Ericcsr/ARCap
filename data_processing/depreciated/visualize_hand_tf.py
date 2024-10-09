import os
import numpy as np
from scipy.spatial.transform import Rotation
from argparse import ArgumentParser
import pybullet as pb
from rigidbodySento import create_primitive_shape
import time
import struct


def compute_rel_transform(pose_base, pose):
    """
    pose_base: np.ndarray shape (7,) [x, y, z, qx, qy, qz, qw] in unity frame
    pose: np.ndarray shape (7,) [x, y, z, qx, qy, qz, qw] in unity frame
    """
    pose_base = pose_base.copy()
    pose_base[:3] = np.array([pose_base[0], pose_base[2], pose_base[1]])
    pose[:3] = np.array([pose[0], pose[2], pose[1]])

    Q = np.array([[1, 0, 0],
                  [0, 0, 1],
                  [0, 1, 0.]])
    rot_base = Rotation.from_quat(pose_base[3:]).as_matrix()
    rot = Rotation.from_quat(pose[3:]).as_matrix()
    rel_rot = Rotation.from_matrix(Q @ (rot_base.T @ rot) @ Q.T) # Is order correct.
    rel_pos = Rotation.from_matrix(Q @ rot_base.T@ Q.T).apply(pose[:3] - pose_base[:3]) # Apply base rotation not relative rotation...
    return rel_pos, rel_rot.as_quat()

def read_floats_from_binary_file(file_path):
    floats = []
    with open(file_path, 'rb') as file:
        # Read the number of floats stored in the first 4 bytes
        num_floats = struct.unpack('I', file.read(4))[0]  # 'I' is for unsigned int
        # Read each float one by one
        for _ in range(num_floats):
            float_data = file.read(4)  # Size of a float in bytes
            if float_data:
                floats.append(struct.unpack('f', float_data)[0])  # 'f' is for float
    return floats

def load_data(folder_path, base_tf):
    files = os.listdir(folder_path)
    files.remove("WorldFrame.txt")
    # Each files is named as "DepthMapxx.bin", need to sort according to xx
    files.sort(key=lambda x: int(x[8:-4]))
    head_tfs = []
    tip_poses = []
    for i,file in enumerate(files):
        raw_data = read_floats_from_binary_file(os.path.join(folder_path, file))
        head_tf = np.array(raw_data[10014:10021]) # In world frame should be move to local frame
        rel_pos, rel_rot = compute_rel_transform(base_tf, head_tf)
        head_tfs.append(np.hstack([rel_pos, rel_rot]))
        tip_pose = np.array(raw_data[10033:10045]).reshape(4,3)[:,[0,2,1]] # In cartesian frame
        tip_pose = tip_pose[[1,2,3,0]]
        tip_pose = Rotation.from_quat(rel_rot).apply(tip_pose) + rel_pos
        tip_poses.append(tip_pose)
    return np.stack(head_tfs), np.stack(tip_poses)

def visualize_tf_traj(tf, o_id):
    pb.resetBasePositionAndOrientation(o_id, tf[:3], tf[3:])

def visualize_fingertip_poses(tip_poses, vis_sp):
    for i, pos in enumerate(tip_poses):
        pb.resetBasePositionAndOrientation(vis_sp[i], pos, (0, 0, 0, 1))

data_folder = "data/2024-05-13-14-54-59"
base_tf = np.genfromtxt(f"{data_folder}/WorldFrame.txt", delimiter=",")
print(base_tf)
c = pb.connect(pb.GUI)
wrist_tfs, hand_tip_poses = load_data(data_folder, base_tf)
np.savez("eef_traj.npz", right_wrist_tfs=wrist_tfs, right_tip_poses=hand_tip_poses)


vis_sp = []
c_code = [[1,0,0,1], [0,1,0,1], [0,0,1,1], [1,1,0,1]]
for i in range(4):
    vis_sp.append(create_primitive_shape(pb, 0.1, pb.GEOM_SPHERE, [0.03], color=c_code[i]))
headset = pb.loadURDF("assets/quest3.urdf")

print(wrist_tfs.shape, hand_tip_poses.shape)

while True:
    for i in range(len(wrist_tfs)):
        print("Reach here")
        visualize_tf_traj(wrist_tfs[i], headset)
        visualize_fingertip_poses(hand_tip_poses[i], vis_sp)
        time.sleep(0.03)
        print(i)
    input("Enter to replay...")
    