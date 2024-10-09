import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
from argparse import ArgumentParser
import struct
import os
import time

# Coordinate frame is y-up, x-right, z-forward
# Point cloud is z-up, x-right, y-forward
def compute_invrel_transform(pose_base, pose):
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
    return rel_pos, rel_rot

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

def build_oobb(A, B, C, D):
    """
    A, B, C, D are the 4 corners of the rectangle, in clockwise order
    """
    # Compute the normal of the rectangle
    normal = -np.cross(B - A, C - B)
    normal -= np.cross(C - B, D - C)
    normal -= np.cross(D - C, A - D)
    normal -= np.cross(A - D, B - A)
    normal /= np.linalg.norm(normal)
    # construct points
    points_ub = np.array([A,B,C,D]) + 0.3 * normal
    points_lb = np.array([A,B,C,D]) - 0.05 * normal
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.vstack((points_ub, points_lb)))
    oobb = pcd.get_oriented_bounding_box()
    return oobb

# Provided FOV is not usable...
z = 0.44
fovLeft = np.arctan(0.57 / z)
fovRight = np.arctan(0.44 / z)
sumud = 1.0
up = 0.5
fovUp = np.arctan(up / z)
fovDown = np.arctan((sumud-up) / z)
# Need to based on the assumption that each pixel is uniformly spaced
def depth_to_pointcloud_naive(depth_image, fovLeft=fovLeft, fovRight=fovRight, fovUp=fovUp, fovDown=fovDown):
    height, width = depth_image.shape
    # Calculate the new principal point based on the shifts
    cx = width * np.tan(fovLeft) / (np.tan(fovLeft) + np.tan(fovRight))
    cy = height * np.tan(fovUp) / (np.tan(fovUp) + np.tan(fovDown))

    # Convert depth image to actual distances (0 to 3 meters)
    depth = depth_image

    # Calculate focal lengths based on field of view
    f_x = cx / np.tan(fovLeft)
    f_y = cy / np.tan(fovUp)
    print(cx, cy)
    # Generate meshgrid for pixel coordinates
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    
    # Transform meshgrid to camera coordinates
    x = (u - cx) * depth / f_x
    y = (v - cy) * depth / f_y
    z = depth
    
    # Flatten arrays
    x = x.flatten()
    y = y.flatten()
    z = z.flatten()

    # Stack into point cloud
    point_cloud = np.vstack((x, z, y)).T
    
    return point_cloud

def load_head_data(folder_path):
    files = os.listdir(folder_path)
    # Each files is named as "DepthMapxx.bin", need to sort according to xx
    files.sort(key=lambda x: int(x[8:-4]))
    point_sets = []
    base_tf = None
    for i,file in enumerate(files):
        raw_data = read_floats_from_binary_file(os.path.join(folder_path, file))
        head_tf = np.array(raw_data[10000:10007])
        if i == 0:
            base_tf = head_tf
        else:
            rel_pos, rel_rot = compute_invrel_transform(base_tf, head_tf)
        image = np.array(raw_data[:10000]).reshape(100, 100)
        points = depth_to_pointcloud_naive(image)
        if i == 0:
            point_sets.append(points)
            continue
        points = (rel_rot @ points.T).T
        points += rel_pos
        point_sets.append(points)
    return point_sets

parser = ArgumentParser()
parser.add_argument("--build_oobb", action="store_true", default=False)
args = parser.parse_args()

point_sets = load_head_data("data/depth_maps")
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(point_sets[0])
print("Number of points: ", point_sets[0])
if args.build_oobb:
    o3d.visualization.draw_geometries_with_vertex_selection([pcd])
    exit()
else:
    point_id = [3447,5353,5377,3286] # TODO:
    A = point_sets[0][point_id[0]]
    B = point_sets[0][point_id[1]]
    C = point_sets[0][point_id[2]]
    D = point_sets[0][point_id[3]]
    oobb = build_oobb(A, B, C, D)
o3d_vis = o3d.visualization.Visualizer()
o3d_vis.create_window()
o3d_vis.add_geometry(pcd)
o3d_vis.add_geometry(oobb)
o3d_vis.run()

input("Press Enter to continue...")
for points in point_sets[1:]:
    pcd.points = o3d.utility.Vector3dVector(points)
    # cropped = pcd.crop(oobb)
    # pcd.points = o3d.utility.Vector3dVector(cropped.points)
    o3d_vis.update_geometry(pcd)
    o3d_vis.poll_events()
    o3d_vis.update_renderer()
    time.sleep(0.1)
