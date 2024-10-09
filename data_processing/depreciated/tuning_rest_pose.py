import pybullet as pb
import numpy as np

c = pb.connect(pb.GUI)

hand_q = np.array([np.pi/6, -np.pi/9, np.pi/6, np.pi/6,
                np.pi/6, 0.0     , np.pi/6, np.pi/6,
                np.pi/6, np.pi/9 , np.pi/6, np.pi/6,
                np.pi/6, np.pi/6 , np.pi/6, np.pi/6])

def set_joint_positions(robot, joint_positions):
    jid = 0
    for i in range(len(joint_positions)):
        if pb.getJointInfo(robot, jid)[2] != pb.JOINT_FIXED:
            pb.resetJointState(robot, jid, joint_positions[i])
        else:
            jid += 1
            pb.resetJointState(robot, jid, joint_positions[i])
        jid += 1

hand = pb.loadURDF("assets/leap_hand/robot_pybullet.urdf")

set_joint_positions(hand, hand_q)

input("Press enter to end...")
