import os
import time
import datetime
import socket
import numpy as np
from scipy.spatial.transform import Rotation
import pybullet as pb
import shutil
from rigidbodySento import create_primitive_shape

# For different robot, just write different QuestRightArmLeapModule classes
class QuestRobotModule:
    def __init__(self,  vr_ip, local_ip, pose_cmd_port, ik_result_port=None):
        self.vr_ip = vr_ip
        self.local_ip = local_ip
        self.pose_cmd_port = pose_cmd_port
        # Quest should send WorldFrame as well as wrist pose via UDP
        self.wrist_listener_s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.wrist_listener_s.bind(("", pose_cmd_port))
        self.wrist_listener_s.setblocking(1)
        self.wrist_listener_s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 0)
        # Initialize ik sender to Quest
        if ik_result_port is not None:
            self.ik_result_s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.ik_result_dest = (vr_ip, ik_result_port)
        else:
            self.ik_result_s = None

    def set_joint_positions(self, robot, joint_positions):
        jid = 0
        for i in range(len(joint_positions)):
            if pb.getJointInfo(robot, jid)[2] != pb.JOINT_FIXED:
                pb.resetJointState(robot, jid, joint_positions[i])
            else:
                jid += 1
                pb.resetJointState(robot, jid, joint_positions[i])
            jid += 1

    def get_joint_limits(self, robot):
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
    
    def compute_rel_transform(self, pose):
        """
        pose: np.ndarray shape (7,) [x, y, z, qx, qy, qz, qw] in unity frame
        """
        world_frame = self.world_frame.copy()
        world_frame[:3] = np.array([world_frame[0], world_frame[2], world_frame[1]])
        pose[:3] = np.array([pose[0], pose[2], pose[1]])

        Q = np.array([[1, 0, 0],
                    [0, 0, 1],
                    [0, 1, 0.]])
        rot_base = Rotation.from_quat(world_frame[3:]).as_matrix()
        rot = Rotation.from_quat(pose[3:]).as_matrix()
        rel_rot = Rotation.from_matrix(Q @ (rot_base.T @ rot) @ Q.T) # Is order correct.
        rel_pos = Rotation.from_matrix(Q @ rot_base.T@ Q.T).apply(pose[:3] - world_frame[:3]) # Apply base rotation not relative rotation...
        return rel_pos, rel_rot.as_quat()
    
    def close(self):
        self.wrist_listener_s.close()
        if self.ik_result_s is not None:
            self.ik_result_s.close()

class QuestRightArmLeapModule(QuestRobotModule):
    ARM_REST = [0.4,
                -0.49826458111314524,
                -0.01990020486871322,
                -2.4732269941140346,
                -0.01307073642274261,
                2.00396583422025,
                1.1980939705504309]

    RIGHT_HAND_Q = [np.pi / 6, -np.pi / 4, np.pi / 3, np.pi / 6,
              np.pi / 6, 0.0, np.pi / 3, np.pi / 6,
              np.pi / 6, np.pi / 4, np.pi / 3, np.pi / 6,
              np.pi / 6, np.pi / 6, np.pi / 3, np.pi / 6]
    fingertip_idx = [4, 9, 14, 19] # Use real fingertip indices

    right_hand_dest = np.array([[0.09, 0.02, -0.1], [0.09, -0.03, -0.1], [0.09, -0.08, -0.1], [0.01, 0.02, -0.14]])
    
    right_hand_mount_offset = [0.05, -0.05, 0.1]

    right_hand_pos_offset = np.array([0.0, 0.0, 0.0]) # -0.03 palm overlap with robot hand, but grasping is hard...
    
    right_hand_orn_offset = Rotation.from_euler("xyz", [-np.pi, 0., 0.])

    right_palm_pos_orn_offset = np.array([-0.1, -0.05, 0.05, 0.0, 0.0, -np.pi/2])
    
    def __init__(self, vr_ip, local_ip, pose_cmd_port, ik_result_port, vis_sp=None):
        super().__init__(vr_ip, local_ip, pose_cmd_port, ik_result_port)
        self.vis_sp = vis_sp
        # Initialize robots
        self.right_arm = pb.loadURDF("assets/franka_arm/panda_leap.urdf", basePosition=[0.0, 0.0, 0.0], baseOrientation=[0, 0, 0.7071068, 0.7071068], useFixedBase=True)
        self.right_hand = pb.loadURDF("assets/leap_hand/robot_pybullet.urdf")
        self.set_joint_positions(self.right_arm, QuestRightArmLeapModule.ARM_REST)
        self.set_joint_positions(self.right_hand, QuestRightArmLeapModule.RIGHT_HAND_Q)
        self.right_lower_limits, self.right_upper_limits, self.right_joint_ranges = self.get_joint_limits(self.right_arm)
        self.right_hand_lower_limits, self.right_hand_upper_limits, self.right_hand_joint_ranges = self.get_joint_limits(self.right_hand)
        self.data_dir = None
        self.prev_data_dir = self.data_dir
        self.last_arm_q = None
        self.last_hand_q = None
    
    def solve_arm_ik(self, wrist_pos, wrist_orn, wrist_offset=None):
        # Solve IK for the wrist position
        if wrist_offset is not None:
            wrist_pos_ = wrist_orn.apply(wrist_offset[:3]) + wrist_pos # In world frame
            wrist_orn_ = wrist_orn * Rotation.from_euler("xyz", wrist_offset[3:])
        else:
            wrist_pos_ = wrist_pos
            wrist_orn_ = wrist_orn
        target_q = pb.calculateInverseKinematics(self.right_arm, 9, wrist_pos_, wrist_orn_.as_quat(), 
                                                 lowerLimits=self.right_lower_limits, upperLimits=self.right_upper_limits, 
                                                 jointRanges=self.right_joint_ranges, restPoses=QuestRightArmLeapModule.ARM_REST, 
                                                 maxNumIterations=40, residualThreshold=0.001)
        return target_q
    
    def solve_fingertip_ik(self, fingertip_pos):
        tip_poses = []
        for i,fid in enumerate(QuestRightArmLeapModule.fingertip_idx):
            tip_pos = fingertip_pos[i]
            if self.vis_sp is not None:
                pb.resetBasePositionAndOrientation(self.vis_sp[i], tip_pos, (0, 0, 0, 1))
            tip_poses.append(tip_pos)
        target_q = []
        for i in range(4):
            target_q = target_q + list(pb.calculateInverseKinematics(self.right_hand, QuestRightArmLeapModule.fingertip_idx[i], tip_poses[i], 
                                                                     lowerLimits=self.right_hand_lower_limits, upperLimits=self.right_hand_upper_limits,
                                                                     jointRanges=self.right_hand_joint_ranges, restPoses=QuestRightArmLeapModule.RIGHT_HAND_Q, 
                                                                     maxNumIterations=40, residualThreshold=0.001))[4*i:4*(i+1)]
        return target_q

    def solve_system_world(self, wrist_pos, wrist_orn, tip_poses):
        arm_q = self.solve_arm_ik(wrist_pos+wrist_orn.apply(QuestRightArmLeapModule.right_hand_pos_offset), wrist_orn * QuestRightArmLeapModule.right_hand_orn_offset.inv(), QuestRightArmLeapModule.right_palm_pos_orn_offset)
        self.set_joint_positions(self.right_arm, arm_q)
        hand_xyz = np.asarray(pb.getLinkState(self.right_arm, 9)[0])
        hand_orn = Rotation.from_quat(pb.getLinkState(self.right_arm, 9)[1])
        pb.resetBasePositionAndOrientation(self.right_hand, hand_xyz + (hand_orn * QuestRightArmLeapModule.right_hand_orn_offset).apply(QuestRightArmLeapModule.right_hand_mount_offset), (hand_orn * QuestRightArmLeapModule.right_hand_orn_offset).as_quat())
        hand_q = self.solve_fingertip_ik(tip_poses)
        self.set_joint_positions(self.right_hand, hand_q)
        self.this_arm_q = arm_q
        self.this_hand_q = hand_q
        return arm_q, hand_q, hand_xyz, hand_orn.as_quat()
    
    def check_delta_joints(self, this_q, prev_q, threshold=0.1):
        if prev_q is None:
            return True
        delta_q = np.abs(np.array(this_q) - np.array(prev_q))
        return np.all(delta_q < threshold)

    # World frame marks beginning of a program.
    def receive(self):
        data, _ = self.wrist_listener_s.recvfrom(1024)
        data_string = data.decode()
        now = datetime.datetime.now()
        #print(data_string)
        if data_string.startswith("WorldFrame"):
            data_string = data_string[11:]
            data_string = data_string.split(",")
            data_list = [float(data) for data in data_string]
            world_frame = np.array(data_list)
            self.world_frame = world_frame
            self.wf_receive_ts = now.strftime("%Y-%m-%d-%H-%M-%S")
            self.set_joint_positions(self.right_arm, QuestRightArmLeapModule.ARM_REST)
            self.set_joint_positions(self.right_hand, QuestRightArmLeapModule.RIGHT_HAND_Q)
            os.mkdir(f"data/{self.wf_receive_ts}")
            np.save(f"data/{self.wf_receive_ts}/WorldFrame.npy", world_frame)
            return None, None
        elif data_string.startswith("Start"):
            formatted_time = now.strftime("%Y-%m-%d-%H-%M-%S")
            self.data_dir = f"data/{self.wf_receive_ts}/{formatted_time}"
            os.mkdir(self.data_dir)
            return None, None
        elif data_string.startswith("Stop"):
            formatted_time = now.strftime("%Y-%m-%d-%H-%M-%S")
            if self.data_dir is not None:
                self.prev_data_dir = self.data_dir
            self.data_dir = None
            return None, None
        elif data_string.startswith("Remove"):
            if self.data_dir is not None and os.path.exists(self.data_dir):
                shutil.rmtree(self.data_dir)
            elif self.prev_data_dir is not None and os.path.exists(self.prev_data_dir):
                shutil.rmtree(self.prev_data_dir)
            self.data_dir = None
            self.prev_data_dir = None
            return None, None
        elif data_string.find("RHand") != -1:
            data_string_ = data_string[7:].split(",")
            data_list = [float(data) for data in data_string_]
            wrist_tf = np.array(data_list[:7])
            head_tf = np.array(data_list[7:])
            rel_wrist_pos, rel_wrist_rot = self.compute_rel_transform(wrist_tf)
            rel_head_pos, rel_head_rot = self.compute_rel_transform(head_tf)
            if self.data_dir is None and data_string[0] == "Y":
                formatted_time = now.strftime("%Y-%m-%d-%H-%M-%S")
                self.data_dir = f"data/{self.wf_receive_ts}/{formatted_time}"
                os.mkdir(self.data_dir)
            return (rel_wrist_pos, rel_wrist_rot), (rel_head_pos, rel_head_rot)
        

    def send_ik_result(self, right_arm_q, right_hand_q):
        delta_result = self.check_delta_joints(right_arm_q, self.last_arm_q) and self.check_delta_joints(right_hand_q, self.last_hand_q, 0.2)
        if self.data_dir is None:
            delta_result = "G"
        elif delta_result:
            delta_result = "Y"
        else:
            delta_result = "N"
        msg = f"{delta_result},{right_arm_q[0]:.3f},{right_arm_q[1]:.3f},{right_arm_q[2]:.3f},{right_arm_q[3]:.3f},{right_arm_q[4]:.3f},{right_arm_q[5]:.3f},{right_arm_q[6]:.3f}"
        msg += f",{right_hand_q[0]:.3f},{right_hand_q[1]:.3f},{right_hand_q[2]:.3f},{right_hand_q[3]:.3f},{right_hand_q[4]:.3f},{right_hand_q[5]:.3f},{right_hand_q[6]:.3f},{right_hand_q[7]:.3f},{right_hand_q[8]:.3f},{right_hand_q[9]:.3f},{right_hand_q[10]:.3f},{right_hand_q[11]:.3f},{right_hand_q[12]:.3f},{right_hand_q[13]:.3f},{right_hand_q[14]:.3f},{right_hand_q[15]:.3f}"
        self.ik_result_s.sendto(msg.encode(), self.ik_result_dest)
        self.last_arm_q = right_arm_q
        self.last_hand_q = right_hand_q
        return right_hand_q

class QuestLeftArmGripperModule(QuestRobotModule):
    ARM_REST = [0.,
                -0.49826458111314524,
                -0.01990020486871322,
                -2.4732269941140346,
                -0.01307073642274261,
                2.00396583422025,
                -0.7227]
    JOINT_DAMPING = [1000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    LEFT_HAND_Q = [-0.04, -0.04]
    left_hand_dest = np.array([[0.09, 0.02, -0.1], [0.09, -0.03, -0.1], [0.09, -0.08, -0.1], [0.01, 0.02, -0.14]])
    
    left_hand_mount_pos_offset = [0, 0, 0]
    
    left_hand_mount_orn_offset = Rotation.from_euler("xyz", [0., 0., 0.])

    left_hand_pos_offset = np.array([0.0, 0.0, 0.0]) # -0.03 palm overlap with robot hand, but grasping is hard...
    
    left_hand_orn_offset = Rotation.from_euler("xyz", [-np.pi, 0., 0.])

    left_palm_pos_orn_offset = np.array([0., 0.0, 0., 0.0, 0.0, np.pi-np.pi/8])
    
    def __init__(self, vr_ip, local_ip, pose_cmd_port, ik_result_port, vis_sp=None):
        super().__init__(vr_ip, local_ip, pose_cmd_port, ik_result_port)
        self.vis_sp = vis_sp
        # Initialize robots
        self.left_arm = pb.loadURDF("assets/franka_arm/panda_gripper.urdf", basePosition=[0.0, 0.0, 0.0], baseOrientation=[0, 0, 0.7071068, 0.7071068], useFixedBase=True)
        self.left_hand = pb.loadURDF("assets/gripper/franka_panda_tri_gripper.urdf")
        self.set_joint_positions(self.left_arm, QuestLeftArmGripperModule.ARM_REST)
        self.set_joint_positions(self.left_hand, QuestLeftArmGripperModule.LEFT_HAND_Q)
        self.left_lower_limits, self.left_upper_limits, self.left_joint_ranges = self.get_joint_limits(self.left_arm)
        self.left_hand_lower_limits, self.left_hand_upper_limits, self.left_hand_joint_ranges = [0.0, 0.0], [0.04, 0.04], [0.04, 0.04]
        self.data_dir = None
        self.prev_data_dir = self.data_dir
        self.last_arm_q = None
        self.last_hand_q = None
        self.last_action = 1
        self.last_action_t = time.time()
    
    def get_triangle_orn(self, index, middle, thumb, wrist):
        origin = (2*thumb + middle+index)/4
        front = (index+middle)/2
        y = front - thumb
        if self.vis_sp is not None:
            pb.resetBasePositionAndOrientation(self.vis_sp[0], thumb, (0, 0, 0, 1))
            pb.resetBasePositionAndOrientation(self.vis_sp[1], front, (0, 0, 0, 1))
            pb.resetBasePositionAndOrientation(self.vis_sp[2], wrist, (0, 0, 0, 1))
        x = np.cross(wrist - origin, y)
        z = np.cross(x, y)
        # normalize
        x = x/np.linalg.norm(x)
        y = y/np.linalg.norm(y)
        z = z/np.linalg.norm(z)
        # Should solve transformation from world frame to this
        orn = Rotation.from_matrix(np.vstack([x, y, z]).T)
        return orn

    def solve_arm_ik(self, wrist_pos, wrist_orn, wrist_offset=None):
        # Solve IK for the wrist position
        if wrist_offset is not None:
            wrist_pos_ = wrist_orn.apply(wrist_offset[:3]) + wrist_pos # In world frame
            wrist_orn_ = wrist_orn * Rotation.from_euler("xyz", wrist_offset[3:])
        else:
            wrist_pos_ = wrist_pos
            wrist_orn_ = wrist_orn
        target_q = pb.calculateInverseKinematics(self.left_arm, 9, wrist_pos_, wrist_orn_.as_quat(), 
                                                 lowerLimits=self.left_lower_limits, upperLimits=self.left_upper_limits, 
                                                 jointRanges=self.left_joint_ranges, restPoses=QuestLeftArmGripperModule.ARM_REST,
                                                 jointDamping=QuestLeftArmGripperModule.JOINT_DAMPING,
                                                 maxNumIterations=40, residualThreshold=0.001)
        #print(target_q[6])
        return target_q
    
    def solve_fingertip_ik(self, fingertip_pos):
        # Should return one joint angle for each finger
        # TODO: Should check whether fingertip_pos is feasible.
        thumb_pos = fingertip_pos[3]
        index_pos = fingertip_pos[0]
        middle_pos = fingertip_pos[1]
        distance = 0.5*np.linalg.norm(thumb_pos - index_pos)+0.5*np.linalg.norm(thumb_pos - middle_pos)
        midpoint_pos = (2*thumb_pos + index_pos + index_pos)/4
        return np.array([distance/2, distance/2]), midpoint_pos

    def solve_system_world(self, wrist_pos, wrist_orn, tip_poses):
        hand_q, midpoint_pos = self.solve_fingertip_ik(tip_poses)
        #gripper_orn = self.get_triangle_orn(tip_poses[0], tip_poses[1], tip_poses[3], (tip_poses[5]+tip_poses[6])*0.5)
        arm_q = self.solve_arm_ik(midpoint_pos, wrist_orn * QuestLeftArmGripperModule.left_hand_orn_offset.inv(), QuestLeftArmGripperModule.left_palm_pos_orn_offset)
        #arm_q = self.solve_arm_ik(midpoint_pos, gripper_orn, QuestRightArmGripperModule.left_palm_pos_orn_offset)
        self.set_joint_positions(self.left_arm, arm_q)
        hand_xyz = np.asarray(pb.getLinkState(self.left_arm, 8)[0])
        hand_orn = Rotation.from_quat(pb.getLinkState(self.left_arm, 8)[1])
        pb.resetBasePositionAndOrientation(self.left_hand, hand_xyz + (hand_orn * QuestLeftArmGripperModule.left_hand_mount_orn_offset).apply(QuestLeftArmGripperModule.left_hand_mount_pos_offset), (hand_orn * QuestLeftArmGripperModule.left_hand_mount_orn_offset).as_quat())
        
        self.set_joint_positions(self.left_hand, hand_q)
        self.this_arm_q = arm_q
        self.this_hand_q = hand_q
        return arm_q, hand_q, np.asarray(pb.getLinkState(self.left_arm, 9)[0]), np.asarray(pb.getLinkState(self.left_arm, 9)[1])
    
    def check_delta_joints(self, this_q, prev_q, threshold=0.1):
        if prev_q is None:
            return True
        delta_q = np.abs(np.array(this_q) - np.array(prev_q))
        return np.all(delta_q < threshold)

    def is_closer_than_threshold(self, hand_q, threshold=None):
        # Check whether the distance between fingers is feasible.
        if threshold is None:
            threshold = self.left_hand_joint_ranges[0]+self.left_hand_joint_ranges[1]
        distance = hand_q[0] + hand_q[1]
        if distance > threshold:
            return False
        return True

    def compute_action(self, this_q): # -1 is open, 1 is close
        #closing = (np.array(this_q) - np.array(prev_q)) < -0.005
        #opening = (np.array(this_q) - np.array(prev_q)) > 0.005
        if time.time() - self.last_action_t < 1.5:
            return self.last_action
        if self.is_closer_than_threshold(this_q, 0.95*(self.left_hand_joint_ranges[0]+self.left_hand_joint_ranges[1])):
            if self.last_action != 1:
                self.last_action_t = time.time()
            self.last_action = 1
            return 1
        else:
            if self.last_action != -1:
                self.last_action_t = time.time()
            self.last_action = -1
            return -1
            
    # World frame marks beginning of a program.
    def receive(self):
        data, _ = self.wrist_listener_s.recvfrom(1024)
        data_string = data.decode()
        now = datetime.datetime.now()
        if data_string.startswith("WorldFrame"):
            data_string = data_string[11:]
            data_string = data_string.split(",")
            data_list = [float(data) for data in data_string]
            world_frame = np.array(data_list)
            self.world_frame = world_frame
            self.wf_receive_ts = now.strftime("%Y-%m-%d-%H-%M-%S")
            self.set_joint_positions(self.left_arm, QuestLeftArmGripperModule.ARM_REST)
            self.set_joint_positions(self.left_hand, QuestLeftArmGripperModule.LEFT_HAND_Q)
            os.mkdir(f"data/{self.wf_receive_ts}")
            np.save(f"data/{self.wf_receive_ts}/WorldFrame.npy", world_frame)
            return None, None
        elif data_string.startswith("Start"):
            formatted_time = now.strftime("%Y-%m-%d-%H-%M-%S")
            self.data_dir = f"data/{self.wf_receive_ts}/{formatted_time}"
            os.mkdir(self.data_dir)
            return None, None
        elif data_string.startswith("Stop"):
            formatted_time = now.strftime("%Y-%m-%d-%H-%M-%S")
            if self.data_dir is not None:
                self.prev_data_dir = self.data_dir
            self.data_dir = None
            return None, None
        elif data_string.startswith("Remove"):
            if self.data_dir is not None and os.path.exists(self.data_dir):
                shutil.rmtree(self.data_dir)
            elif self.prev_data_dir is not None and os.path.exists(self.prev_data_dir):
                shutil.rmtree(self.prev_data_dir)
            self.data_dir = None
            self.prev_data_dir = None
            return None, None
        elif data_string.find("LHand") != -1:
            data_string_ = data_string[7:].split(",")
            data_list = [float(data) for data in data_string_]
            wrist_tf = np.array(data_list[:7])
            head_tf = np.array(data_list[7:])
            rel_wrist_pos, rel_wrist_rot = self.compute_rel_transform(wrist_tf)
            rel_head_pos, rel_head_rot = self.compute_rel_transform(head_tf)
            if self.data_dir is None and data_string[0] == "Y":
                formatted_time = now.strftime("%Y-%m-%d-%H-%M-%S")
                self.data_dir = f"data/{self.wf_receive_ts}/{formatted_time}"
                os.mkdir(self.data_dir)
            return (rel_wrist_pos, rel_wrist_rot), (rel_head_pos, rel_head_rot)
        
    def send_ik_result(self, left_arm_q, left_hand_q):
        delta_result = self.check_delta_joints(left_arm_q, self.last_arm_q) #and self.check_delta_joints(left_hand_q, self.last_hand_q, 0.2) and self.is_closer_than_threshold(left_hand_q)
        action = self.compute_action(left_hand_q)
        hand_q_feedback = np.array([0.0, 0.0]) if action == 1 else np.array([0.04, 0.04])
        if self.data_dir is None:
            delta_result = "G"
        elif delta_result:
            delta_result = "Y"
        else:
            delta_result = "N"
        msg = f"{delta_result},{left_arm_q[0]:.3f},{left_arm_q[1]:.3f},{left_arm_q[2]:.3f},{left_arm_q[3]:.3f},{left_arm_q[4]:.3f},{left_arm_q[5]:.3f},{left_arm_q[6]:.3f}"
        msg += f",{hand_q_feedback[0]:.3f},{hand_q_feedback[1]:.3f}"
        self.ik_result_s.sendto(msg.encode(), self.ik_result_dest)
        self.last_arm_q = left_arm_q
        self.last_hand_q = left_hand_q
        return action


class QuestLeftArmGripperNoRokokoModule(QuestRobotModule):
    ARM_REST = [0.,
                -0.49826458111314524,
                -0.01990020486871322,
                -2.4732269941140346,
                -0.01307073642274261,
                2.00396583422025,
                -0.7227]
    JOINT_DAMPING = [1000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    LEFT_HAND_Q = [-0.04, -0.04]
    left_hand_dest = np.array([[0.09, 0.02, -0.1], [0.09, -0.03, -0.1], [0.09, -0.08, -0.1], [0.01, 0.02, -0.14]])
    
    left_hand_mount_pos_offset = [0, 0, 0]
    
    left_hand_mount_orn_offset = Rotation.from_euler("xyz", [0., 0., 0.])

    left_hand_pos_offset = np.array([0.0, 0.0, 0.0]) # -0.03 palm overlap with robot hand, but grasping is hard...
    
    left_hand_orn_offset = Rotation.from_euler("xyz", [0., 0., 0.])

    left_palm_pos_orn_offset = np.array([0.1, 0.03, 0.04, 0.0, 0.0, -np.pi/2-np.pi/8])
    
    def __init__(self, vr_ip, local_ip, pose_cmd_port, ik_result_port, vis_sp=None):
        super().__init__(vr_ip, local_ip, pose_cmd_port, ik_result_port)
        self.vis_sp = vis_sp
        # Initialize robots
        self.left_arm = pb.loadURDF("assets/franka_arm/panda_gripper.urdf", basePosition=[0.0, 0.0, 0.0], baseOrientation=[0, 0, 0.7071068, 0.7071068], useFixedBase=True)
        self.left_hand = pb.loadURDF("assets/gripper/franka_panda_tri_gripper.urdf")
        self.set_joint_positions(self.left_arm, QuestLeftArmGripperNoRokokoModule.ARM_REST)
        self.set_joint_positions(self.left_hand, QuestLeftArmGripperNoRokokoModule.LEFT_HAND_Q)
        self.left_lower_limits, self.left_upper_limits, self.left_joint_ranges = self.get_joint_limits(self.left_arm)
        self.left_hand_lower_limits, self.left_hand_upper_limits, self.left_hand_joint_ranges = [0.0, 0.0], [0.04, 0.04], [0.04, 0.04]
        self.data_dir = None
        self.prev_data_dir = self.data_dir
        self.last_arm_q = None
        self.last_hand_q = None
        self.last_action = 1
        self.last_action_t = time.time()
    
    def get_triangle_orn(self, index, middle, thumb, wrist):
        origin = (2*thumb + middle+index)/4
        front = (index+middle)/2
        y = front - thumb
        if self.vis_sp is not None:
            pb.resetBasePositionAndOrientation(self.vis_sp[0], thumb, (0, 0, 0, 1))
            pb.resetBasePositionAndOrientation(self.vis_sp[1], front, (0, 0, 0, 1))
            pb.resetBasePositionAndOrientation(self.vis_sp[2], wrist, (0, 0, 0, 1))
        x = np.cross(wrist - origin, y)
        z = np.cross(x, y)
        # normalize
        x = x/np.linalg.norm(x)
        y = y/np.linalg.norm(y)
        z = z/np.linalg.norm(z)
        # Should solve transformation from world frame to this
        orn = Rotation.from_matrix(np.vstack([x, y, z]).T)
        return orn

    def solve_arm_ik(self, wrist_pos, wrist_orn, wrist_offset=None):
        # Solve IK for the wrist position
        if wrist_offset is not None:
            wrist_pos_ = wrist_orn.apply(wrist_offset[:3]) + wrist_pos # In world frame
            wrist_orn_ = wrist_orn * Rotation.from_euler("xyz", wrist_offset[3:])
        else:
            wrist_pos_ = wrist_pos
            wrist_orn_ = wrist_orn
        target_q = pb.calculateInverseKinematics(self.left_arm, 9, wrist_pos_, wrist_orn_.as_quat(), 
                                                 lowerLimits=self.left_lower_limits, upperLimits=self.left_upper_limits, 
                                                 jointRanges=self.left_joint_ranges, restPoses=QuestLeftArmGripperNoRokokoModule.ARM_REST,
                                                 jointDamping=QuestLeftArmGripperNoRokokoModule.JOINT_DAMPING,
                                                 maxNumIterations=40, residualThreshold=0.001)
        #print(target_q[6])
        return target_q
    
    def solve_fingertip_ik(self, fingertip_pos):
        # Should return one joint angle for each finger
        # TODO: Should check whether fingertip_pos is feasible.
        if self.is_open:
            return np.array([0.04, 0.04]), None
        else:
            return np.array([0.0, 0.0]), None

    def solve_system_world(self, wrist_pos, wrist_orn, tip_poses):
        hand_q, midpoint_pos = self.solve_fingertip_ik(tip_poses)
        #gripper_orn = self.get_triangle_orn(tip_poses[0], tip_poses[1], tip_poses[3], (tip_poses[5]+tip_poses[6])*0.5)
        arm_q = self.solve_arm_ik(wrist_pos, wrist_orn * QuestLeftArmGripperNoRokokoModule.left_hand_orn_offset.inv(), QuestLeftArmGripperNoRokokoModule.left_palm_pos_orn_offset)
        #arm_q = self.solve_arm_ik(midpoint_pos, gripper_orn, QuestRightArmGripperModule.left_palm_pos_orn_offset)
        self.set_joint_positions(self.left_arm, arm_q)
        hand_xyz = np.asarray(pb.getLinkState(self.left_arm, 8)[0])
        hand_orn = Rotation.from_quat(pb.getLinkState(self.left_arm, 8)[1])
        pb.resetBasePositionAndOrientation(self.left_hand, hand_xyz + (hand_orn * QuestLeftArmGripperNoRokokoModule.left_hand_mount_orn_offset).apply(QuestLeftArmGripperNoRokokoModule.left_hand_mount_pos_offset), (hand_orn * QuestLeftArmGripperNoRokokoModule.left_hand_mount_orn_offset).as_quat())
        
        self.set_joint_positions(self.left_hand, hand_q)
        self.this_arm_q = arm_q
        self.this_hand_q = hand_q
        return arm_q, hand_q, np.asarray(pb.getLinkState(self.left_arm, 9)[0]), np.asarray(pb.getLinkState(self.left_arm, 9)[1])
    
    def check_delta_joints(self, this_q, prev_q, threshold=0.1):
        if prev_q is None:
            return True
        delta_q = np.abs(np.array(this_q) - np.array(prev_q))
        return np.all(delta_q < threshold)

    def is_closer_than_threshold(self, hand_q, threshold=None):
        # Check whether the distance between fingers is feasible.
        if threshold is None:
            threshold = self.left_hand_joint_ranges[0]+self.left_hand_joint_ranges[1]
        distance = hand_q[0] + hand_q[1]
        if distance > threshold:
            return False
        return True

    def compute_action(self, this_q): # -1 is open, 1 is close
        #closing = (np.array(this_q) - np.array(prev_q)) < -0.005
        #opening = (np.array(this_q) - np.array(prev_q)) > 0.005
        if time.time() - self.last_action_t < 1.5:
            return self.last_action
        if self.is_closer_than_threshold(this_q, 0.95*(self.left_hand_joint_ranges[0]+self.left_hand_joint_ranges[1])):
            if self.last_action != 1:
                self.last_action_t = time.time()
            self.last_action = 1
            return 1
        else:
            if self.last_action != -1:
                self.last_action_t = time.time()
            self.last_action = -1
            return -1
            
    # World frame marks beginning of a program.
    def receive(self):
        data, _ = self.wrist_listener_s.recvfrom(1024)
        data_string = data.decode()
        now = datetime.datetime.now()
        if data_string.startswith("WorldFrame"):
            data_string = data_string[11:]
            data_string = data_string.split(",")
            data_list = [float(data) for data in data_string]
            world_frame = np.array(data_list)
            self.world_frame = world_frame
            self.wf_receive_ts = now.strftime("%Y-%m-%d-%H-%M-%S")
            self.set_joint_positions(self.left_arm, QuestLeftArmGripperNoRokokoModule.ARM_REST)
            self.set_joint_positions(self.left_hand, QuestLeftArmGripperNoRokokoModule.LEFT_HAND_Q)
            os.mkdir(f"data/{self.wf_receive_ts}")
            np.save(f"data/{self.wf_receive_ts}/WorldFrame.npy", world_frame)
            return None, None
        elif data_string.startswith("Start"):
            formatted_time = now.strftime("%Y-%m-%d-%H-%M-%S")
            self.data_dir = f"data/{self.wf_receive_ts}/{formatted_time}"
            os.mkdir(self.data_dir)
            return None, None
        elif data_string.startswith("Stop"):
            formatted_time = now.strftime("%Y-%m-%d-%H-%M-%S")
            if self.data_dir is not None:
                self.prev_data_dir = self.data_dir
            self.data_dir = None
            return None, None
        elif data_string.startswith("Remove"):
            if self.data_dir is not None and os.path.exists(self.data_dir):
                shutil.rmtree(self.data_dir)
            elif self.prev_data_dir is not None and os.path.exists(self.prev_data_dir):
                shutil.rmtree(self.prev_data_dir)
            self.data_dir = None
            self.prev_data_dir = None
            return None, None
        elif data_string.find("LHand") != -1:
            data_string_ = data_string[7:].split(",")
            data_list = [float(data) for data in data_string_]
            wrist_tf = np.array(data_list[:7])
            head_tf = np.array(data_list[7:14])
            if (data_list[14] > 0.1):
                self.is_open = True
            else:
                self.is_open = False
            #print("is open:", self.is_open, data_list[14])
            rel_wrist_pos, rel_wrist_rot = self.compute_rel_transform(wrist_tf)
            rel_head_pos, rel_head_rot = self.compute_rel_transform(head_tf)
            if self.data_dir is None and data_string[0] == "Y":
                formatted_time = now.strftime("%Y-%m-%d-%H-%M-%S")
                self.data_dir = f"data/{self.wf_receive_ts}/{formatted_time}"
                os.mkdir(self.data_dir)
            return (rel_wrist_pos, rel_wrist_rot), (rel_head_pos, rel_head_rot)
        
    def send_ik_result(self, left_arm_q, left_hand_q):
        delta_result = self.check_delta_joints(left_arm_q, self.last_arm_q) #and self.check_delta_joints(left_hand_q, self.last_hand_q, 0.2) and self.is_closer_than_threshold(left_hand_q)
        action = self.compute_action(left_hand_q)
        hand_q_feedback = np.array([0.0, 0.0]) if action == 1 else np.array([0.04, 0.04])
        if self.data_dir is None:
            delta_result = "G"
        elif delta_result:
            delta_result = "Y"
        else:
            delta_result = "N"
        msg = f"{delta_result},{left_arm_q[0]:.3f},{left_arm_q[1]:.3f},{left_arm_q[2]:.3f},{left_arm_q[3]:.3f},{left_arm_q[4]:.3f},{left_arm_q[5]:.3f},{left_arm_q[6]:.3f}"
        msg += f",{hand_q_feedback[0]:.3f},{hand_q_feedback[1]:.3f}"
        self.ik_result_s.sendto(msg.encode(), self.ik_result_dest)
        self.last_arm_q = left_arm_q
        self.last_hand_q = left_hand_q
        return action

class QuestBimanualModule(QuestRobotModule):
    def __init__(self, vr_ip, local_ip, pose_cmd_port, ik_result_port, vis_sp=None):
        super().__init__(vr_ip, local_ip, pose_cmd_port, ik_result_port)
        self.vis_sp = vis_sp
        # Initialize robots
        self.left_arm = pb.loadURDF("assets/franka_arm/panda_gripper.urdf", basePosition=[0.0, 0.0, 0.0], baseOrientation=[0, 0, 0.7071068, 0.7071068], useFixedBase=True)
        self.left_hand = pb.loadURDF("assets/gripper/franka_panda_tri_gripper.urdf")
        self.right_arm = pb.loadURDF("assets/franka_arm/panda_leap.urdf", basePosition=[0.0, 0.0, 0.0], baseOrientation=[0, 0, 0.7071068, 0.7071068], useFixedBase=True)
        self.right_hand = pb.loadURDF("assets/leap_hand/robot_pybullet.urdf")
        self.set_joint_positions(self.left_arm, QuestLeftArmGripperModule.ARM_REST)
        self.set_joint_positions(self.left_hand, QuestLeftArmGripperModule.LEFT_HAND_Q)
        self.set_joint_positions(self.right_arm, QuestRightArmLeapModule.ARM_REST)
        self.set_joint_positions(self.right_hand, QuestRightArmLeapModule.RIGHT_HAND_Q)

        self.left_lower_limits, self.left_upper_limits, self.left_joint_ranges = self.get_joint_limits(self.left_arm)
        self.right_lower_limits, self.right_upper_limits, self.right_joint_ranges = self.get_joint_limits(self.right_arm)
        self.right_hand_lower_limits, self.right_hand_upper_limits, self.right_hand_joint_ranges = self.get_joint_limits(self.right_hand)
        self.left_hand_lower_limits, self.left_hand_upper_limits, self.left_hand_joint_ranges = [0.0, 0.0], [0.04, 0.04], [0.04, 0.04]
        self.data_dir = None
        self.prev_data_dir = self.data_dir
        self.last_arm_q = None
        self.last_hand_q = None
        self.last_action = 1
        self.last_action_t = time.time()

    def solve_arm_ik(self, wrist_pos, wrist_orn, wrist_offset=None, handedness="left"):
        if wrist_offset is not None:
            wrist_pos_ = wrist_orn.apply(wrist_offset[:3]) + wrist_pos # In world frame
            wrist_orn_ = wrist_orn * Rotation.from_euler("xyz", wrist_offset[3:])
        else:
            wrist_pos_ = wrist_pos
            wrist_orn_ = wrist_orn
        if handedness == "left":
            target_q = pb.calculateInverseKinematics(self.left_arm, 9, wrist_pos_, wrist_orn_.as_quat(), 
                                                 lowerLimits=self.left_lower_limits, upperLimits=self.left_upper_limits, 
                                                 jointRanges=self.left_joint_ranges, restPoses=QuestLeftArmGripperModule.ARM_REST,
                                                 jointDamping=QuestLeftArmGripperModule.JOINT_DAMPING,
                                                 maxNumIterations=30, residualThreshold=0.005)
        else:
            target_q = pb.calculateInverseKinematics(self.right_arm, 9, wrist_pos_, wrist_orn_.as_quat(), 
                                                    lowerLimits=self.right_lower_limits, upperLimits=self.right_upper_limits, 
                                                    jointRanges=self.right_joint_ranges, restPoses=QuestRightArmLeapModule.ARM_REST, 
                                                    maxNumIterations=30, residualThreshold=0.005)
        return target_q
    
    def solve_fingertip_ik(self, fingertip_pos):
        tip_poses = []
        for i,fid in enumerate(QuestRightArmLeapModule.fingertip_idx):
            tip_pos = fingertip_pos[i]
            if self.vis_sp is not None:
                pb.resetBasePositionAndOrientation(self.vis_sp[i], tip_pos, (0, 0, 0, 1))
            tip_poses.append(tip_pos)
        target_q = []
        for i in range(4):
            target_q = target_q + list(pb.calculateInverseKinematics(self.right_hand, QuestRightArmLeapModule.fingertip_idx[i], tip_poses[i], 
                                                                     lowerLimits=self.right_hand_lower_limits, upperLimits=self.right_hand_upper_limits,
                                                                     jointRanges=self.right_hand_joint_ranges, restPoses=QuestRightArmLeapModule.RIGHT_HAND_Q, 
                                                                     maxNumIterations=40, residualThreshold=0.002))[4*i:4*(i+1)]
        return target_q
    
    def solve_gripper_ik(self, fingertip_pos):
        thumb_pos = fingertip_pos[3]
        index_pos = fingertip_pos[0]
        middle_pos = fingertip_pos[1]
        distance = 0.5*np.linalg.norm(thumb_pos - index_pos)+0.5*np.linalg.norm(thumb_pos - middle_pos)
        midpoint_pos = (2*thumb_pos + index_pos + index_pos)/4
        return np.array([distance/2, distance/2]), midpoint_pos
    
    def solve_system_world(self, 
                           left_wrist_pos, left_wrist_orn,
                           right_wrist_pos, right_wrist_orn,
                           left_tip_poses, right_tip_poses):
        # left arm and hand
        left_hand_q, left_midpoint_pos = self.solve_gripper_ik(left_tip_poses)
        left_arm_q = self.solve_arm_ik(left_midpoint_pos, left_wrist_orn * QuestLeftArmGripperModule.left_hand_orn_offset.inv(), QuestLeftArmGripperModule.left_palm_pos_orn_offset, handedness="left")
        self.set_joint_positions(self.left_arm, left_arm_q)
        left_hand_xyz = np.asarray(pb.getLinkState(self.left_arm, 8)[0])
        left_hand_orn = Rotation.from_quat(pb.getLinkState(self.left_arm, 8)[1])
        pb.resetBasePositionAndOrientation(self.left_hand, left_hand_xyz + (left_hand_orn * QuestLeftArmGripperModule.left_hand_mount_orn_offset).apply(QuestLeftArmGripperModule.left_hand_mount_pos_offset), (left_hand_orn * QuestLeftArmGripperModule.left_hand_mount_orn_offset).as_quat())
        self.set_joint_positions(self.left_hand, left_hand_q)
        # right arm and hand
        right_arm_q = self.solve_arm_ik(right_wrist_pos+right_wrist_orn.apply(QuestRightArmLeapModule.right_hand_pos_offset), right_wrist_orn * QuestRightArmLeapModule.right_hand_orn_offset.inv(), QuestRightArmLeapModule.right_palm_pos_orn_offset, handedness="right")
        self.set_joint_positions(self.right_arm, right_arm_q)
        right_hand_xyz = np.asarray(pb.getLinkState(self.right_arm, 9)[0])
        right_hand_orn = Rotation.from_quat(pb.getLinkState(self.right_arm, 9)[1])
        pb.resetBasePositionAndOrientation(self.right_hand, right_hand_xyz + (right_hand_orn * QuestRightArmLeapModule.right_hand_orn_offset).apply(QuestRightArmLeapModule.right_hand_mount_offset), (right_hand_orn * QuestRightArmLeapModule.right_hand_orn_offset).as_quat())
        right_hand_q = self.solve_fingertip_ik(right_tip_poses)
        self.set_joint_positions(self.right_hand, right_hand_q)

        # Store current joint angles
        self.this_arm_q = np.hstack([left_arm_q, right_arm_q])
        self.this_hand_q = np.hstack([left_hand_q, right_hand_q])

        left_hand_xyz = np.asarray(pb.getLinkState(self.left_arm, 9)[0])
        left_hand_orn = np.asarray(pb.getLinkState(self.left_arm, 9)[1])

        return self.this_arm_q, self.this_hand_q, np.hstack([left_hand_xyz, right_hand_xyz]), np.hstack([left_hand_orn, right_hand_orn.as_quat()])
    
    def check_delta_joints(self, this_q, prev_q, threshold=0.1):
        if prev_q is None:
            return True
        delta_q = np.abs(np.array(this_q) - np.array(prev_q))
        return np.all(delta_q < threshold)
    
    def is_closer_than_threshold(self, hand_q, threshold=None):
        # Check whether the distance between fingers is feasible.
        if threshold is None:
            threshold = self.left_hand_joint_ranges[0]+self.left_hand_joint_ranges[1]
        distance = hand_q[0] + hand_q[1]
        if distance > threshold:
            return False
        return True
    
    def compute_action(self, this_q):
        if time.time() - self.last_action_t < 1.5:
            return self.last_action
        if self.is_closer_than_threshold(this_q, 0.95*(self.left_hand_joint_ranges[0]+self.left_hand_joint_ranges[1])):
            if self.last_action != 1:
                self.last_action_t = time.time()
            self.last_action = 1
            return 1
        else:
            if self.last_action != -1:
                self.last_action_t = time.time()
            self.last_action = -1
            return -1

    def receive(self):
        data, _ = self.wrist_listener_s.recvfrom(1024)
        data_string = data.decode()
        #print(data_string)
        now = datetime.datetime.now()
        if data_string.startswith("WorldFrame"):
            data_string = data_string[11:]
            data_string = data_string.split(",")
            data_list = [float(data) for data in data_string]
            world_frame = np.array(data_list)
            self.world_frame = world_frame
            self.wf_receive_ts = now.strftime("%Y-%m-%d-%H-%M-%S")
            self.set_joint_positions(self.right_arm, QuestRightArmLeapModule.ARM_REST)
            self.set_joint_positions(self.right_hand, QuestRightArmLeapModule.RIGHT_HAND_Q)
            os.mkdir(f"data/{self.wf_receive_ts}")
            np.save(f"data/{self.wf_receive_ts}/WorldFrame.npy", world_frame)
            return None, None, None
        elif data_string.startswith("RobotFrame"):
            print("Robot frame received")
            data_string = data_string[11:]
            data_string = data_string.split(",")
            data_list = [float(data) for data in data_string]
            robot_frame = np.array(data_list)
            self.set_joint_positions(self.left_arm, QuestLeftArmGripperModule.ARM_REST)
            self.set_joint_positions(self.left_hand, QuestLeftArmGripperModule.LEFT_HAND_Q)
            robot_pos, robot_orn = self.compute_rel_transform(robot_frame)
            print("Robot pos:", robot_pos)
            np.save(f"data/{self.wf_receive_ts}/RobotFrame.npy", np.hstack([robot_pos, robot_orn]))
            pb.resetBasePositionAndOrientation(self.left_arm, robot_pos, (Rotation.from_quat(robot_orn)*Rotation.from_quat([0, 0, 0.7071068, 0.7071068])).as_quat())
            return None, None, None
        elif data_string.startswith("Start"):
            formatted_time = now.strftime("%Y-%m-%d-%H-%M-%S")
            self.data_dir = f"data/{self.wf_receive_ts}/{formatted_time}"
            os.mkdir(self.data_dir)
            return None, None, None
        elif data_string.startswith("Stop"):
            formatted_time = now.strftime("%Y-%m-%d-%H-%M-%S")
            if self.data_dir is not None:
                self.prev_data_dir = self.data_dir
            self.data_dir = None
            return None, None, None
        elif data_string.startswith("Remove"):
            if self.data_dir is not None and os.path.exists(self.data_dir):
                shutil.rmtree(self.data_dir)
            elif self.prev_data_dir is not None and os.path.exists(self.prev_data_dir):
                shutil.rmtree(self.prev_data_dir)
            self.data_dir = None
            self.prev_data_dir = None
            return None, None, None
        elif data_string.find("BHand") != -1:
            data_string_ = data_string[7:].split(",")
            data_list = [float(data) for data in data_string_]
            left_wrist_tf = np.array(data_list[:7])
            right_wrist_tf = np.array(data_list[7:14])
            head_tf = np.array(data_list[14:])
            rel_left_wrist_pos, rel_left_wrist_rot = self.compute_rel_transform(left_wrist_tf)
            rel_right_wrist_pos, rel_right_wrist_rot = self.compute_rel_transform(right_wrist_tf)
            rel_head_pos, rel_head_rot = self.compute_rel_transform(head_tf)
            if self.data_dir is None and data_string[0] == "Y":
                formatted_time = now.strftime("%Y-%m-%d-%H-%M-%S")
                self.data_dir = f"data/{self.wf_receive_ts}/{formatted_time}"
                os.mkdir(self.data_dir)
            return (rel_left_wrist_pos, rel_left_wrist_rot), (rel_right_wrist_pos, rel_right_wrist_rot), (rel_head_pos, rel_head_rot)
        
    def send_ik_result(self, arm_q, hand_q):
        delta_result = self.check_delta_joints(arm_q, self.last_arm_q) #and self.check_delta_joints(left_hand_q, self.last_hand_q, 0.2) and self.is_closer_than_threshold(left_hand_q)
        action = self.compute_action(hand_q[:2]) # left hand action
        hand_q_feedback = np.array([0.0, 0.0]) if action == 1 else np.array([0.04, 0.04])
        if self.data_dir is None:
            delta_result = "G"
        elif delta_result:
            delta_result = "Y"
        else:
            delta_result = "N"
        msg = f"{delta_result}"
        for i in range(len(arm_q)):
            msg += f",{arm_q[i]:.3f}"
        msg += f",{hand_q_feedback[0]:.3f},{hand_q_feedback[1]:.3f}"
        for i in range(2,len(hand_q)):
            msg += f",{hand_q[i]:.3f}"
        self.ik_result_s.sendto(msg.encode(), self.ik_result_dest)
        self.last_arm_q = arm_q
        self.last_hand_q = hand_q
        return np.hstack([action, hand_q[2:]])