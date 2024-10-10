import socket
import time
from argparse import ArgumentParser
import numpy as np
from scipy.spatial.transform import Rotation
import pybullet as pb
from rigidbodySento import create_primitive_shape
from ip_config import *
from rokoko_module import RokokoModule
from realsense_module import DepthCameraModule
from quest_robot_module import QuestBimanualModule

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--frequency", type=int, default=20)
    parser.add_argument("--no_camera", action="store_true", default=False)
    args = parser.parse_args()
    c = pb.connect(pb.DIRECT)
    vis_sp = []
    c_code = c_code = [[1,0,0,1], [0,1,0,1], [0,0,1,1], [1,1,0,1]]
    for i in range(4):
        vis_sp.append(create_primitive_shape(pb, 0.1, pb.GEOM_SPHERE, [0.02], color=c_code[i]))
    if not args.no_camera:
        camera = DepthCameraModule(is_decimate=False, visualize=False)
    rokoko = RokokoModule(VR_HOST, HAND_INFO_PORT, ROKOKO_PORT)
    quest = QuestBimanualModule(VR_HOST, LOCAL_HOST, POSE_CMD_PORT, IK_RESULT_PORT, vis_sp=None)

    start_time = time.time()
    fps_counter = 0
    packet_counter = 0
    print("Initialization completed")
    current_ts = time.time()
    while True:
        now = time.time()
        # TODO: May cause communication issues, need to tune on AR side.
        if now - current_ts < 1 / args.frequency: 
            continue
        else:
            current_ts = now
        try:
            if not args.no_camera:
                point_cloud = camera.receive()
            left_positions, right_positions = rokoko.receive()
            rokoko.send_joint_data(np.vstack([left_positions[:5], right_positions[:5]]))
            wrist_left, wrist_right, head_pose= quest.receive()
            if wrist_left is not None and wrist_right is not None:
                # Left hand as gripper
                wrist_left_orn = Rotation.from_quat(wrist_left[1])
                wrist_left_pos = wrist_left[0]
                # Right hand as leap hand
                wrist_right_orn = Rotation.from_quat(wrist_right[1])
                wrist_right_pos = wrist_right[0]
                head_pos = head_pose[0]
                head_orn = Rotation.from_quat(head_pose[1])
                right_tip_pose = wrist_right_orn.apply(right_positions) + wrist_right_pos                
                left_tip_pose = wrist_left_orn.apply(left_positions) + wrist_left_pos
                right_tip_pose[[0,1,2,3]] = right_tip_pose[[1,2,3,0]]
                left_tip_pose[[0,1,2,3]] = left_tip_pose[[1,2,3,0]]
                arm_q, hand_q, wrist_pos, wrist_orn = quest.solve_system_world(wrist_left_pos, wrist_left_orn, 
                                                                               wrist_right_pos, wrist_right_orn,
                                                                               left_tip_pose, right_tip_pose) # arm_q and hand_q should be concatenation of left and right
                action = quest.send_ik_result(arm_q, hand_q)
                if quest.data_dir is not None:
                    if args.no_camera:
                        point_cloud = np.zeros((1000,3)) # dummy point cloud
                    np.savez(f"{quest.data_dir}/both_data_{time.time()}.npz", right_wrist_pos=wrist_pos, right_wrist_orn=wrist_orn,
                                                                            left_wrist_pos=wrist_pos, left_wrist_orn=wrist_orn, 
                                                                            head_pos=head_pos, head_orn=head_orn.as_quat(),
                                                                            arm_q=arm_q, hand_q=action,raw_hand_q=hand_q,
                                                                            right_tip_poses=right_tip_pose, 
                                                                            left_tip_poses=left_tip_pose,
                                                                            point_cloud=point_cloud)
        except socket.error as e:
            print(e)
            pass
        except KeyboardInterrupt:
            if not args.no_camera:
                camera.close()
            rokoko.close()
            quest.close()
            break
        else:
            packet_time = time.time()
            fps_counter += 1
            packet_counter += 1

            if (packet_time - start_time) > 1.0:
                print(f"received {fps_counter} packets in a second", end="\r")
                start_time += 1.0
                fps_counter = 0
        

