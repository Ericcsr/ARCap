import pybullet as pb
import numpy as np
from scipy.spatial.transform import Rotation
from rigidbodySento import create_primitive_shape
import time
import sys
import socket
from ip_config import *
c = pb.connect(pb.GUI)

hand_pos_offset = [0.05, -0.05, 0.1]
hand_orn_offset = Rotation.from_euler("xyz", [-np.pi,0,0])

palm_orn_offset = Rotation.from_euler("xyz", [0,0,-np.pi/2])

arm = pb.loadURDF("assets/franka_arm/panda.urdf", basePosition=[0.0, 0.0, 0.0], baseOrientation=[0, 0, 0.7071068, 0.7071068], useFixedBase=True)
hand = pb.loadURDF("assets/leap_hand/robot_pybullet.urdf")


RIGHT_REST = [0.4,
            -0.49826458111314524,
            -0.01990020486871322,
            -2.4732269941140346,
            -0.01307073642274261,
            2.00396583422025,
            1.1980939705504309]

HAND_Q = np.array([np.pi / 6, -np.pi / 6, np.pi / 3, np.pi / 6,
                   np.pi / 6, 0.0, np.pi / 3, np.pi / 6,
                   np.pi / 6, np.pi / 6, np.pi / 3, np.pi / 6,
                   np.pi / 6, np.pi / 6, np.pi / 3, np.pi / 6])

fingertip_idx = [4, 9, 14, 19] # Use real fingertip indices

hand_dest = np.array([[0.09, 0.02, -0.1], [0.09, -0.03, -0.1], [0.09, -0.08, -0.1], [0.01, 0.02, -0.14]])

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

def set_joint_positions(robot, joint_positions):
    jid = 0
    for i in range(len(joint_positions)):
        if pb.getJointInfo(robot, jid)[2] != pb.JOINT_FIXED:
            pb.resetJointState(robot, jid, joint_positions[i])
        else:
            jid += 1
            pb.resetJointState(robot, jid, joint_positions[i])
        jid += 1

c_code = [[1,0,0,1], [0,1,0,1], [0,0,1,1], [1,1,0,1]]

def solve_fingertip_ik(hand, fingertip_pos, palm_orn_offset=None, palm_pos_offset=None, vis_sp=None):
    """
    If palm_orn_offset and palm_pos_offset are provided fingertip_pose in local frame
    Otherwise fingertip_pose is in world frame
    """
    tip_poses = []
    for i,fid in enumerate(fingertip_idx):
        tip_pos = fingertip_pos[i]
        if palm_orn_offset is not None or palm_pos_offset is not None:
            tip_pos = palm_orn_offset.apply(tip_pos) + palm_pos_offset # Should be in world frame
            # Visualize tip position
        #create_primitive_shape(pb, 0.1, pb.GEOM_SPHERE, [0.02],init_xyz=tip_pos, color = c_code[i])
        if vis_sp is not None:
            pb.resetBasePositionAndOrientation(vis_sp[i], tip_pos, (0, 0, 0, 1))
        tip_poses.append(tip_pos)
    target_q = []
    for i in range(4):
        target_q = target_q + list(pb.calculateInverseKinematics(hand, fingertip_idx[i], tip_poses[i], lowerLimits=hand_lower_limits, upperLimits=hand_upper_limits, jointRanges = hand_joint_ranges, restPoses = HAND_Q.tolist(), maxNumIterations=100, residualThreshold=0.001))[4*i:4*(i+1)]
    return target_q

def solve_arm_ik(arm, wrist_pos, wrist_orn, wrist_offset=None):
    # Solve IK for the wrist position
    if wrist_offset is not None:
        wrist_pos_ = wrist_orn.apply(wrist_offset[:3]) + wrist_pos # In world frame
        wrist_orn = wrist_orn * Rotation.from_euler("xyz", wrist_offset[3:])
    target_q = pb.calculateInverseKinematics(arm, 9, wrist_pos_, wrist_orn.as_quat(), lowerLimits=right_lower_limits, upperLimits=right_upper_limits, jointRanges=right_joint_ranges, restPoses=RIGHT_REST, maxNumIterations=100, residualThreshold=0.001)
    return target_q

def set_system_local(arm, hand, wrist_pos, wrist_orn, wrist_offset, tip_poses):
    arm_q = solve_arm_ik(arm, wrist_pos, wrist_orn * hand_orn_offset.inv(), wrist_offset)
    set_joint_positions(arm, arm_q)
    hand_xyz = np.asarray(pb.getLinkState(arm, 9)[0])
    hand_orn = Rotation.from_quat(pb.getLinkState(arm, 9)[1])
    pb.resetBasePositionAndOrientation(hand, hand_xyz + (hand_orn * hand_orn_offset).apply(hand_pos_offset), (hand_orn * hand_orn_offset).as_quat())
    hand_q = solve_fingertip_ik(hand, tip_poses, wrist_orn, wrist_pos)
    set_joint_positions(hand, hand_q)
    return arm_q, hand_q

def set_system_world(arm, hand, wrist_pos, wrist_orn, wrist_offset, tip_poses, vis_sp=None):
    arm_q = solve_arm_ik(arm, wrist_pos, wrist_orn * hand_orn_offset.inv(), wrist_offset)
    set_joint_positions(arm, arm_q)
    hand_xyz = np.asarray(pb.getLinkState(arm, 9)[0])
    hand_orn = Rotation.from_quat(pb.getLinkState(arm, 9)[1])
    pb.resetBasePositionAndOrientation(hand, hand_xyz + (hand_orn * hand_orn_offset).apply(hand_pos_offset), (hand_orn * hand_orn_offset).as_quat())
    hand_q = solve_fingertip_ik(hand, tip_poses, vis_sp=vis_sp)
    set_joint_positions(hand, hand_q)
    return arm_q, hand_q

def get_joint_limits(robot):
    joint_lower_limits = []
    joint_upper_limits = []
    joint_ranges = []
    for i in range(pb.getNumJoints(robot)):
        joint_info = pb.getJointInfo(robot, i)
        if joint_info[2] == pb.JOINT_FIXED:
            continue
        joint_lower_limits.append(joint_info[8])
        joint_upper_limits.append(joint_info[9])
        joint_ranges.append(joint_info[9] - joint_info[8])
    return joint_lower_limits, joint_upper_limits, joint_ranges

eef_data = np.load("data/eef_traj.npz")
wrist_tfs = eef_data["right_wrist_tfs"]
hand_tip_poses = eef_data["right_tip_poses"]

right_lower_limits, right_upper_limits, right_joint_ranges = get_joint_limits(arm)
hand_lower_limits, hand_upper_limits, hand_joint_ranges = get_joint_limits(hand)
print(len(hand_lower_limits), len(hand_upper_limits), len(hand_joint_ranges))

vis_sp = []
for i in range(4):
    vis_sp.append(create_primitive_shape(pb, 0.1, pb.GEOM_SPHERE, [0.02], color=c_code[i]))

print("Creating broadcast socket")
joint_s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
addr = (VR_HOST, IK_RESULT_PORT)

print("Connecting to Quest")
listener_s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
listener_s.bind(("", POSE_CMD_PORT))
listener_s.setblocking(1)
listener_s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2048)
set_joint_positions(arm, RIGHT_REST)
set_joint_positions(hand, HAND_Q)
arm_traj = []
hand_traj = []

start_time = time.time()
fps_counter = 0
packet_counter = 0

while True:
    try:
        # Check if data is available
        if listener_s.recv != None:
            data, address = listener_s.recvfrom(1024)
            data_string = data.decode()
            if data_string.startswith("RHand"): # Should be 
                data_string = data_string[6:].split(",")
                data_list = [float(data) for data in data_string]
                wrist_tf = np.array(data_list[:7]) # Only right wrist for now.
                rel_pos, rel_rot = compute_rel_transform(world_frame, wrist_tf)
                if len(data_string) > 7:
                    hand_tip_pose = np.array(data_list[7:]).reshape(4, 3)[[1,2,3,0]] # thumb comes last
                    hand_tip_pose = hand_tip_pose[:, [0, 2, 1]]
                    hand_tip_pose = Rotation.from_quat(rel_rot).apply(hand_tip_pose) + rel_pos
                else:
                    hand_tip_pose = hand_tip_poses[0]
                # Set fingertip poses zero
                arm_q, hand_q = set_system_world(arm, hand, rel_pos, Rotation.from_quat(rel_rot), np.array([-0.1, -0.05, 0.05, 0.0, 0.0, -np.pi/2]), hand_tip_pose, vis_sp)
                arm_traj.append(arm_q)
                hand_traj.append(hand_q)
                # Only send arm data for now.
                data = f"{arm_q[0]:.3f},{arm_q[1]:.3f},{arm_q[2]:.3f},{arm_q[3]:.3f},{arm_q[4]:.3f},{arm_q[5]:.3f},{arm_q[6]:.3f}"
                data += f",{hand_q[0]:.3f},{hand_q[1]:.3f},{hand_q[2]:.3f},{hand_q[3]:.3f},{hand_q[4]:.3f},{hand_q[5]:.3f},{hand_q[6]:.3f},{hand_q[7]:.3f},{hand_q[8]:.3f},{hand_q[9]:.3f},{hand_q[10]:.3f},{hand_q[11]:.3f},{hand_q[12]:.3f},{hand_q[13]:.3f},{hand_q[14]:.3f},{hand_q[15]:.3f}"
                joint_s.sendto(data.encode(), addr)
                #time.sleep(0.05)
            elif data_string.startswith("WorldFrame"):
                data_string = data_string[11:]
                data_string = data_string.split(",")
                data_list = [float(data) for data in data_string]
                world_frame = np.array(data_list)
                set_joint_positions(hand, HAND_Q)
                set_joint_positions(arm, RIGHT_REST)
                print("World Frame received!")
    except socket.error as e:
        print(e)
        pass
    except KeyboardInterrupt:
        listener_s.close()
        joint_s.close()
        sys.exit()
    else:
        packet_time = time.time()
        fps_counter += 1
        packet_counter += 1

        if (packet_time - start_time) > 1.0:
            print(f"received {fps_counter} packets in a second", end="\r")
            start_time += 1.0
            fps_counter = 0
    
    