import sys
import socket
import time
import json
import numpy as np
from scipy.spatial.transform import Rotation
from ip_config import *

HOST, PORT = "", 14043
UNPACK_PACKET = False
SAVE_TO_FILE = False

hand_link_names = ["Hand",
                   "ThumbProximal","ThumbMedial","ThumbDistal","ThumbTip",
                   "IndexProximal","IndexMedial","IndexDistal","IndexTip",
                   "MiddleProximal","MiddleMedial","MiddleDistal","MiddleTip",
                   "RingProximal","RingMedial","RingDistal","RingTip",
                   "LittleProximal","LittleMedial","LittleDistal","LittleTip"]

tip_id = [4, 8, 12, 16, 20]


def round_floats(o):
    if isinstance(o, float):
        return round(o, 5)
    if isinstance(o, dict):
        return {k: round_floats(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [round_floats(x) for x in o]
    return o

def Unpack(data):
    info = json.loads(data)
    left_link_positions = []
    if "leftHand" in info["scene"]["actors"][0]["body"].keys():
        raw_wrist_orn = info["scene"]["actors"][0]["body"]["leftHand"]["rotation"]
        wrist_orn = Rotation.from_quat(np.array([raw_wrist_orn["x"], raw_wrist_orn["y"], raw_wrist_orn["z"], raw_wrist_orn["w"]]))
        raw_wrist_position = info["scene"]["actors"][0]["body"]["leftHand"]["position"]
        wrist_position = np.array([raw_wrist_position["x"], raw_wrist_position["y"], raw_wrist_position["z"]])
        for link_name in hand_link_names:
            link_name = "left" + link_name
            raw_pos = info["scene"]["actors"][0]["body"][link_name]["position"]
            pos = np.array([raw_pos["x"], raw_pos["y"], raw_pos["z"]])
            rel_pos = wrist_orn.inv().apply(pos - wrist_position)
            left_link_positions.append(rel_pos)
        left_link_positions = -np.array(left_link_positions)
    right_link_positions = []
    if "rightHand" in info["scene"]["actors"][0]["body"].keys():
        raw_wrist_orn = info["scene"]["actors"][0]["body"]["rightHand"]["rotation"]
        wrist_orn = Rotation.from_quat(np.array([raw_wrist_orn["x"], raw_wrist_orn["y"], raw_wrist_orn["z"], raw_wrist_orn["w"]]))
        raw_wrist_position = info["scene"]["actors"][0]["body"]["rightHand"]["position"]
        wrist_position = np.array([raw_wrist_position["x"], raw_wrist_position["y"], raw_wrist_position["z"]])
        for link_name in hand_link_names:
            link_name = "right" + link_name
            raw_pos = info["scene"]["actors"][0]["body"][link_name]["position"]
            pos = np.array([raw_pos["x"], raw_pos["y"], raw_pos["z"]])
            rel_pos = wrist_orn.inv().apply(pos - wrist_position)
            right_link_positions.append(rel_pos)
        right_link_positions = -np.array(right_link_positions)
    return left_link_positions[tip_id], right_link_positions[tip_id]

def send_joint_data(tip_positions, socket, address):
    """
    tip_positions: list of lists of 3 floats
    conn: socket connector
    """
    msg = {}
    msg["lt"] = tip_positions[0].tolist()
    msg["li"] = tip_positions[1].tolist()
    msg["lm"] = tip_positions[2].tolist()
    msg["lr"] = tip_positions[3].tolist()
    msg["rt"] = tip_positions[5].tolist()
    msg["ri"] = tip_positions[6].tolist()
    msg["rm"] = tip_positions[7].tolist()
    msg["rr"] = tip_positions[8].tolist()
    json_message = json.dumps(round_floats(msg))
    socket.sendto(json_message.encode(), address)

start_time = time.time()
fps_counter = 0
packet_counter = 0

print("Connecting to Rokoko")
rokoko_s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rokoko_s.bind(("", ROKOKO_PORT))
rokoko_s.setblocking(1)

print("Connecting to quest")
quest_s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
addr = (VR_HOST, HAND_BC_PORT)

while True:
    try:
        data, address = rokoko_s.recvfrom(40000)
        left_positions, right_positions = Unpack(data)
        if len(left_positions) > 0 and len(right_positions) > 0:
            joint_positions = np.vstack([left_positions, right_positions])
            send_joint_data(joint_positions, quest_s, addr)
        else:
            print("No hand data received")
    except socket.error as e:
        print(e)
        pass
    except KeyboardInterrupt:
        rokoko_s.close()
        quest_s.close()
        sys.exit()
    else:
        packet_time = time.time()
        fps_counter += 1
        packet_counter += 1

        if (packet_time - start_time) > 1.0:
            print(f"received {fps_counter} packets in a second", end="\r")
            start_time += 1.0
            fps_counter = 0