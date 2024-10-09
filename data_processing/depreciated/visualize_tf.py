import os
import numpy as np
from scipy.spatial.transform import Rotation
from argparse import ArgumentParser
import pybullet as pb
import time
import struct


def compute_rel_transform(pose_base, pose):
    """
    pose_base: np.ndarray shape (7,) [x, y, z, qx, qy, qz, qw]
    pose: np.ndarray shape (7,) [x, y, z, qx, qy, qz, qw]
    """
    pose_base = pose_base.copy()
    pose_base[:3] = np.array([pose_base[0], pose_base[2], pose_base[1]])
    pose[:3] = np.array([pose[0], pose[2], pose[1]])

    Q = np.array([[1, 0, 0],
                  [0, 0, 1],
                  [0, 1, 0.]])
    rot_base = Rotation.from_quat(pose_base[3:]).as_matrix()
    rot = Rotation.from_quat(pose[3:]).as_matrix()
    rel_rot = Q @ (rot_base.T @ rot) @ Q.T # Is order correct.
    rel_pos = pose[:3] - pose_base[:3]
    return rel_pos, Rotation.from_matrix(rel_rot).as_quat()

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

def load_head_tf(folder_path):
    files = os.listdir(folder_path)
    # Each files is named as "DepthMapxx.bin", need to sort according to xx
    files.sort(key=lambda x: int(x[8:-4]))
    head_tfs = []
    base_tf = None
    for i,file in enumerate(files):
        raw_data = read_floats_from_binary_file(os.path.join(folder_path, file))
        head_tf = np.array(raw_data[10000:10007])
        if i == 0:
            base_tf = head_tf
            head_tfs.append(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]))
        else:
            rel_pos, rel_rot = compute_rel_transform(base_tf, head_tf)
            head_tfs.append(np.hstack([rel_pos, rel_rot]))
    return head_tfs

def visualize_tf_traj(head_tfs, o_id):
    for tf in head_tfs:
        print(tf[:3])
        pb.resetBasePositionAndOrientation(o_id, tf[:3], tf[3:])
        time.sleep(0.1)


data_folder = "data/depth_maps"
head_tfs = load_head_tf(data_folder)
pb.connect(pb.GUI)
headset = pb.loadURDF("assets/quest3.urdf")

while True:
    visualize_tf_traj(head_tfs, 0)
    input("Enter to replay...")
    