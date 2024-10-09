import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import cv2
from argparse import ArgumentParser
import struct

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
    return -rel_pos, rel_rot.T

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

# Should accept fovLeft, fovRight, fovUp, fovDown
def depth_to_pointcloud_segmented(depth_image, fovLeft=1.376382, fovRight=0.8390996, fovUp=0.9656888, fovDown=1.428148):
    """
    Convert a depth image to a 3D point cloud, with the camera center not necessarily at the image center.
    
    :param depth_image: numpy array, a normalized depth image with values from 0 to 1
    :param fovLeft: float, field of view in rad
    :param fovRight: float, field of view in rad
    :param fovUp: float, field of view in rad
    :param fovDown: float, field of view in rad
    :param cx: float, x-coordinate of the principal point (camera center), defaults to image center
    :param cy: float, y-coordinate of the principal point, defaults to image center
    :return: numpy array, point cloud of shape (N, 3)
    """
        # Image dimensions
    height, width = depth_image.shape

    # Default principal point to image center if not specified
    cx = width / 2
    cy = height / 2

    # Convert depth image to actual distances (0 to 3 meters)
    depth = depth_image

    # Calculate focal lengths for each direction
    fl_x_left = (cx) / (2 * np.tan((fovLeft / 2)))
    fl_x_right = (width - cx) / (2 * np.tan((fovRight / 2)))
    fl_y_up = (cy) / (2 * np.tan((fovUp / 2)))
    fl_y_down = (height - cy) / (2 * np.tan((fovDown / 2)))

    # Generate meshgrid for pixel coordinates
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    
    # Calculate different focal lengths based on pixel position
    fl_x = np.where(u < cx, fl_x_left, fl_x_right)
    fl_y = np.where(v < cy, fl_y_up, fl_y_down)

    # Transform meshgrid to camera coordinates
    x = (u - cx) * depth / fl_x
    y = (v - cy) * depth / fl_y
    z = depth
    
    # Flatten arrays
    x = x.flatten()
    y = y.flatten()
    z = z.flatten()

    # Stack into point cloud
    point_cloud = np.vstack((x, y, z)).T
    
    return point_cloud

# Provided FOV is not usable...
h_factor = 0.79
v_factor = 0.83
# Need to based on the assumption that each pixel is uniformly spaced
def depth_to_pointcloud_naive(depth_image, fovLeft=1.176382 * h_factor, fovRight=0.9390996 * h_factor, fovUp=0.9656888 * v_factor, fovDown=1.028148 * v_factor):
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

contn = [False]
def Cnt(x):
    contn[0] = True

color_code = [[1,0,0],[0,1,0],[0,0,1],[1,1,0],[1,0,1],[0,1,1]]

def paint_points(pcds):
    colors = []
    for i,points in enumerate(pcds):
        colors.append(np.array([color_code[i % 6]] * points.shape[0]))
    return np.vstack(colors)

parser = ArgumentParser()
parser.add_argument('--depth_id', action='append', required=True, type=int)
parser.add_argument('--save_base', action='store_true', default=False)
args = parser.parse_args()
head_poses = []
pcds = []
rel_poses = []
rel_rots = []
images = []
if not args.save_base:
    head_base = np.load('head_base.npy')
for i in args.depth_id:
    raw_data = read_floats_from_binary_file(f"data/depth_maps/DepthMap{i}.bin")
    head_pose = raw_data[10000:]
    if args.save_base:
        np.save('head_base.npy', head_pose)
        exit(1) # Only use first depth id
    else:
        rel_pos, rel_rot = compute_invrel_transform(head_base, head_pose)
        rel_poses.append(rel_pos)
        rel_rots.append(rel_rot)
        image = np.array(raw_data[:10000]).reshape(100,100)
        #image = np.flipud(image)
        images.append(image)
        points = depth_to_pointcloud_naive(image)
        points = (rel_rot.T @ points.T).T
        points -= rel_pos
        pcds.append(points)

cv2.namedWindow('image')
cv2.createTrackbar('fovLeft', 'image', 53, 180, Cnt)
cv2.createTrackbar('fovRight', 'image', 43, 180, Cnt)
cv2.createTrackbar('fovUp', 'image', 46, 180, Cnt)
cv2.createTrackbar('fovDown', 'image', 49, 180, Cnt)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.vstack(pcds))
pcd.colors = o3d.utility.Vector3dVector(paint_points(pcds))
cropped = pcd.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=(-5, 0, -5), max_bound=(5, 1, 5)))
o3d_vis = o3d.visualization.Visualizer()
o3d_vis.create_window()
o3d_vis.add_geometry(cropped)
# Create a coordinate frame
mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0, 0, 0])
o3d_vis.add_geometry(mesh_frame)
o3d_vis.run()

while True:
    # Convert to y-up, x-right, z-forward
    if contn[0]:
        contn[0] = False
        fovLeft = np.deg2rad(cv2.getTrackbarPos('fovLeft', 'image'))
        fovRight = np.deg2rad(cv2.getTrackbarPos('fovRight', 'image'))
        fovUp = np.deg2rad(cv2.getTrackbarPos('fovUp', 'image'))
        fovDown = np.deg2rad(cv2.getTrackbarPos('fovDown', 'image'))
        for i in range(len(pcds)):
            points = depth_to_pointcloud_naive(images[i], fovLeft, fovRight, fovUp, fovDown)
            points = (rel_rots[i] @ points.T).T
            points += rel_poses[i]
            pcds[i] = points
        pcd.points = o3d.utility.Vector3dVector(np.vstack(pcds))
        pcd.colors = o3d.utility.Vector3dVector(paint_points(pcds))
        # remove background point
        cropped_ = pcd.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=(-5, 0, -5), max_bound=(5, 1, 5)))
        cropped.points = cropped_.points
        cropped.colors = cropped_.colors
        o3d_vis.update_geometry(cropped)
        o3d_vis.poll_events()
        o3d_vis.update_renderer()
    else:
        pass
    cv2.waitKey(100)