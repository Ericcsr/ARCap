import numpy as np
import open3d as o3d
import pybullet as pb
from scipy.spatial.transform import Rotation
from argparse import ArgumentParser
import os
import time

c = pb.connect(pb.GUI)
right_hand_pos_offset = [0.0, 0.0, 0.0]
    
right_hand_orn_offset = Rotation.from_euler("xyz", [0., 0., 0.])

parser = ArgumentParser()
parser.add_argument("--filename", type=str, required=True)
args = parser.parse_args()

def set_joint_positions(robot, joint_positions):
    jid = 0
    for i in range(len(joint_positions)):
            if pb.getJointInfo(robot, jid)[2] != pb.JOINT_FIXED:
                pb.resetJointState(robot, jid, joint_positions[i])
            else:
                jid += 1
                pb.resetJointState(robot, jid, joint_positions[i])
            jid += 1

def set_system_pose(hand, arm, arm_q, hand_q):
    print(arm_q)
    set_joint_positions(arm, arm_q)
    hand_xyz = np.asarray(pb.getLinkState(arm, 8)[0])
    hand_orn = Rotation.from_quat(pb.getLinkState(arm, 8)[1])
    pb.resetBasePositionAndOrientation(hand, hand_xyz + (hand_orn * right_hand_orn_offset).apply(right_hand_pos_offset), (hand_orn * right_hand_orn_offset).as_quat())
    set_joint_positions(hand, hand_q)


def read_data(filename):
    traj = np.load(f"trajs/{filename}.npz")
    for i in range(traj["arm_qs"].shape[0]):
        hand_q = np.array([0., 0.]) if traj["hand_qs"][i] > 0.0 else np.array([0.04, 0.04])
        set_system_pose(hand, arm, traj["arm_qs"][i], hand_q)
        time.sleep(0.01)


# Create virtual robot and arm sample point cloud from head pose Should make leap hand black
arm = pb.loadURDF("assets/franka_arm/panda.urdf", basePosition=[0.0, 0.0, 0.0], baseOrientation=[0, 0, 0.7071068, 0.7071068], useFixedBase=True)
hand = pb.loadURDF("assets/gripper/franka_panda_tri_gripper.urdf")

while True:
    read_data(args.filename)



