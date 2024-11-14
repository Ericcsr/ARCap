import numpy as np
import open3d as o3d
import pybullet as pb
from scipy.spatial.transform import Rotation
from argparse import ArgumentParser
import os
import time

c = pb.connect(pb.GUI)
right_hand_pos_offset = [0.05, -0.05, 0.1]
    
right_hand_orn_offset = Rotation.from_euler("xyz", [-np.pi, 0., 0.])

right_palm_orn_offset = np.array([-0.1, -0.05, 0.05, 0.0, 0.0, -np.pi/2])

IMAGE_SHAPE = [200,200]

FAR = 1.0

NEAR = 0.01

PROJ_MAT = pb.computeProjectionMatrixFOV(fov=90, aspect=1.0, nearVal=NEAR, farVal=FAR)

def getIntrinsicParams(proj_mat, w,h):
    f_x = proj_mat[0] * w / 2
    f_y = proj_mat[5] * h / 2
    c_x = (-proj_mat[2] * w + w) / 2
    c_y = (proj_mat[6] * h + h) / 2
    return np.array([w, h, f_x, f_y, c_x, c_y])

intrinsic = getIntrinsicParams(PROJ_MAT, IMAGE_SHAPE[0], IMAGE_SHAPE[1])

parser = ArgumentParser()
parser.add_argument("--root_path", type=str, required=True, action="append")
parser.add_argument("--stride", type=int, required=False, action="append")
parser.add_argument("--gap", type=int, default=1)
parser.add_argument("--visualize", action="store_true", default=False)
parser.add_argument("--start_idx", type=int, default=0)
parser.add_argument("--x_offset", type=float, default=0.0)
parser.add_argument("--y_offset", type=float, default=0.0)
parser.add_argument("--z_offset", type=float, default=0.0)
parser.add_argument("--use_gripper", action="store_true", default=False)
args = parser.parse_args()

#assert len(args.root_path) == len(args.stride)

def to_world(pcd, head_pos, head_orn, delta_orn, delta_pos):
    pcd_pos = pcd[:,:3]
    pcd_color = pcd[:,3:]
    pcd_world = head_orn.apply(delta_orn.apply(pcd_pos) + delta_pos) + head_pos
    return np.hstack((pcd_world, pcd_color))

def crop_pcd(pcd, min_bound, max_bound):
    pcd_pos = pcd[:,:3]
    pcd_color = pcd[:,3:]
    mask = np.logical_and(np.all(pcd_pos >= min_bound, axis=1), np.all(pcd_pos <= max_bound, axis=1))
    pcd_pos = pcd_pos[mask]
    pcd_color = pcd_color[mask]
    return np.hstack((pcd_pos, pcd_color))

def remove_box(pcd, min_bound, max_bound):
    pcd_pos = pcd[:,:3]
    pcd_color = pcd[:,3:]
    mask = np.logical_or(np.any(pcd_pos < min_bound, axis=1), np.any(pcd_pos > max_bound, axis=1))
    pcd_pos = pcd_pos[mask]
    pcd_color = pcd_color[mask]
    return np.hstack((pcd_pos, pcd_color))

def set_joint_positions(robot, joint_positions):
    jid = 0
    for i in range(len(joint_positions)):
            if pb.getJointInfo(robot, jid)[2] != pb.JOINT_FIXED:
                pb.resetJointState(robot, jid, joint_positions[i])
            else:
                jid += 1
                pb.resetJointState(robot, jid, joint_positions[i])
            jid += 1

def compute_rel_transform(pose, world_frame):
    """
    pose: np.ndarray shape (7,) [x, y, z, qx, qy, qz, qw] in unity frame
    """
    world_frame = world_frame.copy()
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

def set_system_pose(hand, arm, arm_q, hand_q):
    set_joint_positions(arm, arm_q)
    if not args.use_gripper:
        hand_xyz = np.asarray(pb.getLinkState(arm, 9)[0])
        hand_orn = Rotation.from_quat(pb.getLinkState(arm, 9)[1])
        pb.resetBasePositionAndOrientation(hand, hand_xyz + (hand_orn * right_hand_orn_offset).apply(right_hand_pos_offset), (hand_orn * right_hand_orn_offset).as_quat())
    else:
        hand_xyz = np.asarray(pb.getLinkState(arm, 8)[0])
        hand_orn = Rotation.from_quat(pb.getLinkState(arm, 8)[1])
        pb.resetBasePositionAndOrientation(hand, hand_xyz, hand_orn.as_quat())
    set_joint_positions(hand, hand_q)
    return hand_xyz + hand_orn.apply(np.array([0.0, 0.12, -0.05]))

def get_rgbd_image(projectionMat, viewMat):
    img = pb.getCameraImage(IMAGE_SHAPE[0], IMAGE_SHAPE[1], viewMatrix=viewMat, projectionMatrix=projectionMat)
    if not isinstance(img[3], np.ndarray):
        img[3] = np.array(img[3]).reshape(IMAGE_SHAPE)
        img[2] = np.array(img[2]).reshape(IMAGE_SHAPE[0],IMAGE_SHAPE[1],3)
    depth_image = NEAR * FAR /(FAR - (FAR - NEAR)*img[3])
    rgb_image = np.asarray(img[2])[:,:,:3]
    depth_image = (depth_image * 1000).astype(np.uint16)
    return depth_image, rgb_image

# Assume system already in place
def get_system_virtual_pcd(viewMat):
    depth_image, color_image = get_rgbd_image(PROJ_MAT, viewMat)
    pcd = o3d.geometry.PointCloud.create_from_depth_image(o3d.geometry.Image(depth_image), 
                                                          o3d.camera.PinholeCameraIntrinsic(int(intrinsic[0]), int(intrinsic[1]),*intrinsic[2:]),
                                                          depth_scale = 1000.0, depth_trunc = FAR+0.1)
    points = np.asarray(pcd.points) * np.array([1, -1, -1])
    colors = color_image.reshape(-1,3) / 255.0
    points = points
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.transform(np.linalg.inv(np.asarray(viewMat).reshape(4,4).T))
    points = np.asarray(pcd.points)
    colors = colors
    return np.hstack((points, colors))

def process_wrist_pos(wrist_pos, wrist_orn):
    wrist_orn_ = wrist_orn * right_hand_orn_offset.inv()
    wrist_pos_ = wrist_orn_.apply(right_palm_orn_offset[:3]) + wrist_pos
    wrist_orn_ = wrist_orn_ * Rotation.from_euler("xyz",right_palm_orn_offset[3:])
    return wrist_pos_, wrist_orn_.as_quat()

def read_data(path, target_path, delta_orn, delta_pos, stride):
    files = os.listdir(path)
    # Sort files by time stamp right_data_{time.time()}.npz
    files.sort(key=lambda x: float(x.split("_")[2].split(".")[0]))
    bb_lb = np.array([-0.05, -0.2, -0.05])
    bb_ub = np.array([0.05, 0.05, 0.1])
    init_center = (bb_lb + bb_ub) / 2
    if args.visualize:
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        track_bb = o3d.geometry.AxisAlignedBoundingBox(bb_lb, bb_ub)
    first = True
    viewMat = pb.computeViewMatrix(cameraEyePosition=[-0.4, 0.0, 0.3], cameraTargetPosition=[-0.4, 0.8, -0.2], cameraUpVector=[0.0, 0.0, 1.0])
    print("Demo length:",len(files))
    for i,file in enumerate(files):
        handedness = file.split("_")[0]
        if i == 0 and handedness == "left":
            viewMat = pb.computeViewMatrix(cameraEyePosition=[0.4, 0.0, 0.3], cameraTargetPosition=[0.4, 0.8, -0.2], cameraUpVector=[0.0, 0.0, 1.0])
        if i % args.gap != 0:
            continue
        data = np.load(f"{path}/{file}")
        pcd = data["point_cloud"][::stride]
        head_pos, head_orn = data["head_pos"], data["head_orn"]
        world_pcd = to_world(pcd, head_pos, Rotation.from_quat(head_orn), delta_orn, delta_pos)
        world_pcd[:,0] += args.x_offset
        world_pcd[:,1] += args.y_offset
        world_pcd[:,2] += args.z_offset
        # Set robot state
        hand_xyz = set_system_pose(hand, arm, data[f"{handedness}_arm_q"], data["raw_hand_q"])
        robot_pcd = get_system_virtual_pcd(viewMat)
        wrist_pos, wrist_orn = process_wrist_pos(data[f"{handedness}_wrist_pos"], Rotation.from_quat(data[f"{handedness}_wrist_orn"]))
        # remove points near
        #hand_xyz += np.array([])
        if args.visualize:
            track_bb.translate(hand_xyz - (track_bb.get_center()-init_center))
        #world_pcd  = remove_box(world_pcd, hand_xyz + bb_lb , hand_xyz + bb_ub)
        if first:
            vis_pcd = o3d.geometry.PointCloud()
            frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
            if handedness == "right":
                world_pcd = crop_pcd(world_pcd, (-0.4,0.1, 0.01), (-0.1,0.6,0.4))
                robot_pcd = crop_pcd(robot_pcd, (-0.4,0.1, 0.01), (-0.1,0.6,0.4))
            else:
                world_pcd = crop_pcd(world_pcd, (0.1,0.1, 0.005), (0.7,0.3,0.4))
                robot_pcd = crop_pcd(robot_pcd, (0.1,0.1, 0.005), (0.7,0.3,0.4))
            vis_pcd.points = o3d.utility.Vector3dVector(np.vstack([world_pcd[:,:3], robot_pcd[:,:3]]))
            vis_pcd.colors = o3d.utility.Vector3dVector(np.vstack([world_pcd[:,3:], robot_pcd[:,3:]]))
            if args.visualize:
                vis.add_geometry(vis_pcd)
                vis.add_geometry(frame)
                vis.add_geometry(track_bb)
            first = False
        else:
            if handedness == "right":
                world_pcd = crop_pcd(world_pcd, (-0.4,0.1, 0.01), (-0.1,0.6,0.4))
                robot_pcd = crop_pcd(robot_pcd, (-0.4,0.1, 0.01), (-0.1,0.6,0.4))
            else:
                world_pcd = crop_pcd(world_pcd, (0.1,0.1, 0.005), (0.7,0.3,0.4))
                robot_pcd = crop_pcd(robot_pcd, (0.1,0.1, 0.005), (0.7,0.3,0.4))
            vis_pcd.points = o3d.utility.Vector3dVector(np.vstack([world_pcd[:,:3], robot_pcd[:,:3]]))
            vis_pcd.colors = o3d.utility.Vector3dVector(np.vstack([world_pcd[:,3:], robot_pcd[:,3:]]))
            if args.visualize:
                vis.update_geometry(vis_pcd)
                vis.update_geometry(track_bb)
                vis.poll_events()
                vis.update_renderer()
        if not os.path.exists(f"{target_path}/frame_{i}"):
            os.mkdir(f"{target_path}/frame_{i}")
        o3d.io.write_point_cloud(f"{target_path}/frame_{i}/point_cloud.ply", vis_pcd)
        np.savetxt(f"{target_path}/frame_{i}/arm_joints.txt", data[f"{handedness}_arm_q"], delimiter=",")
        np.savetxt(f"{target_path}/frame_{i}/hand_joints.txt", np.atleast_1d(data[f"{handedness}_hand_q"]), delimiter=",")
        np.savetxt(f"{target_path}/frame_{i}/wrist_pos.txt", wrist_pos, delimiter=",")
        np.savetxt(f"{target_path}/frame_{i}/wrist_orn.txt", wrist_orn, delimiter=",")
    if args.visualize:
        vis.destroy_window()

depth_tf = np.load("configs/tf_camera.npz")
depth_R = Rotation.from_matrix(depth_tf["R"])
depth_t = depth_tf["t"]
quest_tf = np.load("configs/calibration.npz")
quest_R = Rotation.from_quat(quest_tf["rel_rot"])
quest_t = quest_tf["rel_pos"]
delta_orn = quest_R.inv() * depth_R
delta_pos = quest_R.inv().apply(depth_t - quest_t)

if os.path.isdir("data_processed"):
    os.mkdir("data_processed")

# Create virtual robot and arm sample point cloud from head pose Should make leap hand black
if not args.use_gripper:
    arm = pb.loadURDF("assets/franka_arm/panda_leap.urdf", basePosition=[0.0, 0.0, 0.0], baseOrientation=[0, 0, 0.7071068, 0.7071068], useFixedBase=True)
    hand = pb.loadURDF("assets/leap_hand/robot_pybullet.urdf")
else:
    arm = pb.loadURDF("assets/franka_arm/panda_gripper.urdf", basePosition=[0.0, 0.0, 0.0], baseOrientation=[0, 0, 0.7071068, 0.7071068], useFixedBase=True)
    hand = pb.loadURDF("assets/gripper/franka_panda_tri_gripper.urdf")

for j, root_path in enumerate(args.root_path):
    print("Processing root path:", root_path)
    folders = os.listdir(f"data/{root_path}")
    folders.remove("WorldFrame.npy")
    folders.sort()
    if not os.path.exists(f"data_processed/{root_path}"):
        os.mkdir(f"data_processed/{root_path}")
    for i,folder in enumerate(folders):
        if i < args.start_idx:
            continue
        target_path = f"data_processed/{root_path}/demo_{i}"
        if not os.path.exists(target_path):
            os.mkdir(target_path)
        read_data(f"data/{root_path}/{folder}", target_path, delta_orn, delta_pos, 1)
        print(f"Processed: demo_{i}")

