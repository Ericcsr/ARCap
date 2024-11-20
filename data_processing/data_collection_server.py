import socket
import zmq
import pickle
import time
from argparse import ArgumentParser
import numpy as np
from scipy.spatial.transform import Rotation
import pybullet as pb
from rigidbodySento import create_primitive_shape
from ip_config import *
from rokoko_module import RokokoModule
from realsense_module import DepthCameraModule
from quest_robot_module import QuestRightArmLeapModule, QuestLeftArmGripperModule

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--frequency", type=int, default=30)
    parser.add_argument("--handedness", type=str, default="right")
    parser.add_argument("--no_camera", action="store_true", default=False)
    parser.add_argument("--to_remote", action="store_true", default=False)
    args = parser.parse_args()
    c = pb.connect(pb.DIRECT)
    vis_sp = []
    c_code = c_code = [[1,0,0,1], [0,1,0,1], [0,0,1,1], [1,1,0,1]]
    for i in range(4):
        vis_sp.append(create_primitive_shape(pb, 0.1, pb.GEOM_SPHERE, [0.02], color=c_code[i]))
    if not args.no_camera:
        camera = DepthCameraModule(is_decimate=False, visualize=False)
    rokoko = RokokoModule(VR_HOST, HAND_INFO_PORT, ROKOKO_PORT)
    if args.handedness == "right":
        quest = QuestRightArmLeapModule(VR_HOST, LOCAL_HOST, POSE_CMD_PORT, IK_RESULT_PORT, vis_sp=None)
    else:
        quest = QuestLeftArmGripperModule(VR_HOST, LOCAL_HOST, POSE_CMD_PORT, IK_RESULT_PORT, vis_sp=vis_sp)

    start_time = time.time()
    fps_counter = 0
    packet_counter = 0
    print("Initialization completed")
    current_ts = time.time()

    if args.to_remote:
        context = zmq.Context()
        zmq_socket = context.socket(zmq.REQ)
        zmq_socket.connect("tcp://10.108.5.247:5555")
        arm_qs = []
        hand_qs = []
    is_alive = False
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
            wrist, head_pose= quest.receive()
            if wrist is not None:
                wrist_orn = Rotation.from_quat(wrist[1])
                wrist_pos = wrist[0]
                head_pos = head_pose[0]
                head_orn = Rotation.from_quat(head_pose[1])
                if args.handedness == "right":
                    hand_tip_pose = wrist_orn.apply(right_positions) + wrist_pos
                else:
                    hand_tip_pose = wrist_orn.apply(left_positions) + wrist_pos
                hand_tip_pose[[0,1,2,3]] = hand_tip_pose[[1,2,3,0]]
                arm_q, hand_q, wrist_pos, wrist_orn = quest.solve_system_world(wrist_pos, wrist_orn, hand_tip_pose)
                action = quest.send_ik_result(arm_q, hand_q)
                #print("ik result sent")
                if quest.data_dir is not None:
                    if is_alive == False:
                        is_alive = True
                        print("start recording")
                    if args.no_camera:
                        point_cloud = np.zeros((1000,3)) # dummy point cloud
                    if args.handedness == "right":
                        if args.to_remote:
                            arm_qs.append(arm_q)
                            hand_qs.append(action)
                        np.savez(f"{quest.data_dir}/right_data_{time.time()}.npz", right_wrist_pos=wrist_pos, right_wrist_orn=wrist_orn, 
                                                                                head_pos=head_pos, head_orn=head_orn.as_quat(),
                                                                                right_arm_q=arm_q, right_hand_q=action,raw_hand_q=hand_q,
                                                                                right_tip_poses=hand_tip_pose, point_cloud=point_cloud)
                    else:
                        if args.to_remote:
                            arm_qs.append(arm_q)
                            hand_qs.append(action)
                        np.savez(f"{quest.data_dir}/left_data_{time.time()}.npz", left_wrist_pos=wrist_pos, left_wrist_orn=wrist_orn, 
                                                                                head_pos=head_pos, head_orn=head_orn.as_quat(),
                                                                                left_arm_q=arm_q, left_hand_q=action, raw_hand_q=hand_q,
                                                                                left_tip_poses=hand_tip_pose, point_cloud=point_cloud)
                else:
                    if is_alive:
                        is_alive = False
                        print("Current recording end")
                        if args.to_remote:
                            # if not deleted
                            if not quest.removed:
                                message = {"arm_qs":np.vstack(arm_qs), "hand_qs":np.stack(hand_qs), "is_end": False}
                                zmq_socket.send(pickle.dumps(message))
                                reply = zmq_socket.recv()
                            arm_qs = []
                            hand_qs = []
        except socket.error as e:
            print(e)
            pass
        except KeyboardInterrupt:
            if not args.no_camera:
                camera.close()
            rokoko.close()
            if args.to_remote:
                message = {"arm_qs":np.vstack(arm_qs), "hand_qs":np.stack(hand_qs), "is_end": True}
                zmq_socket.send(pickle.dumps(message))
                zmq_socket.close()
            quest.close()
            break
        else:
            packet_time = time.time()
            fps_counter += 1
            packet_counter += 1

            if (packet_time - start_time) > 1.0:
                #print(f"received {fps_counter} packets in a second", end="\r")
                start_time += 1.0
                fps_counter = 0
        

