import pyrealsense2 as rs
import socket
import numpy as np
import open3d as o3d
import time
import copy
import pickle
from argparse import ArgumentParser
from ip_config import *

parser = ArgumentParser()
parser.add_argument("--mode", type=str, default="calibration") # ["calibration", "capture", "stream"]
parser.add_argument("--visualize", action="store_true", default=False)
parser.add_argument("--camera",type=str, default="515") # ["455", "435"]
args = parser.parse_args()

pipeline = rs.pipeline()
config = rs.config()

pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("Camera error or not connected!")
    exit(0)

config.enable_stream(rs.stream.depth, rs.format.z16, 30)
config.enable_stream(rs.stream.color, rs.format.rgb8, 30)

pipeline.start(config)
depth_sensor = device.query_sensors()[0]
if depth_sensor.supports(rs.option.emitter_enabled):
    depth_sensor.set_option(rs.option.emitter_enabled, 1)

# get active camera profile and camera intrinsic
profile = pipeline.get_active_profile()
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()
w, h = depth_intrinsics.width, depth_intrinsics.height

# processing blocks
pc = rs.pointcloud()
# TODO: May be not needed
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 4 ** 1)

colorizer = rs.colorizer()

align_to = rs.stream.depth

align = rs.align(align_to)
i = 0

if args.mode == "stream":
    if args.visualize:
        vis = o3d.visualization.Visualizer()
        vis.create_window()

first_run=True

# Initialize socket listener
# listener_s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# listener_s.bind((VR_HOST, DEPTH_CMD_PORT))

time_ts = time.time()
while True:
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    # Get aligned frames
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    # if args.camera == "415":
    #depth_frame = decimate.process(depth_frame)
    #color_frame = decimate.process(color_frame)

    depth_intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()

    w,h = depth_intrinsics.width, depth_intrinsics.height

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    depth_colormap = np.asanyarray(colorizer.colorize(depth_frame).get_data())
    
    mapped_frame, color_source = color_frame, color_image

    points = pc.calculate(depth_frame)
    pc.map_to(mapped_frame)
    v,t = points.get_vertices(), points.get_texture_coordinates()
    verts = np.asanyarray(v).view(np.float32).reshape(-1,3) # xyz
    textcoords = np.asanyarray(t).view(np.float32).reshape(-1,2)
    colors = np.asanyarray(color_frame.get_data(), np.float32).reshape(-1,3) / 255.0

    # THIS IS SLOW
    if args.mode == "stream":
        vis_verts = verts[::20]
        vis_colors = colors[::20]
    else:
        vis_verts = verts
        vis_colors = colors
    # TODO: Apply transformation on point cloud
    o3d_pcd = o3d.geometry.PointCloud()
    o3d_pcd.points = o3d.utility.Vector3dVector(vis_verts)
    o3d_pcd.colors = o3d.utility.Vector3dVector(vis_colors)
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0,0,0])
    if args.mode == "capture": # NOTE: capture mode shouldn't be used for this project
        if i == 0:
            i+=1
            continue
        i+=1
        time.sleep(0.1)
        if i == 10:
            o3d.io.write_point_cloud(f"obj_{args.camera}.ply", o3d_pcd)
            break
    elif args.mode == "calibration":
        cropped = o3d_pcd.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=(-1.5, -1.5, 0.0), max_bound=(1.5, 1.5, 1.5)))
        o3d.visualization.draw_geometries_with_vertex_selection([cropped])
        o3d.io.write_point_cloud(f"calib_data/calib_{args.camera}.ply", cropped)
    elif args.mode == "stream":
        # Receive command from VR
        #o3d_pcd.rotate(eR, center = (0,0,0))
        #o3d_pcd.translate(et)
        cropped = o3d_pcd#.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=(-0.3, -0.3, 0.0), max_bound=(0.3, 0.3, 0.3)))
        pcd_array = np.hstack([np.asarray(cropped.points), np.asarray(cropped.colors)])
        if first_run and args.visualize:
            vis_pcd = copy.deepcopy(cropped)
            vis.add_geometry(vis_pcd)
            vis.add_geometry(frame)
            first_run = False
        elif args.visualize:
            vis_pcd.points = o3d.utility.Vector3dVector(np.asarray(cropped.points))
            vis_pcd.colors = o3d.utility.Vector3dVector(np.asarray(cropped.colors))
            vis.update_geometry(vis_pcd)
            vis.poll_events()
            vis.update_renderer()
    print("FPS: ", 1/(time.time()-time_ts), "Number of points:", len(verts))
    np.save(f"data/dump/point_cloud{time_ts}.npy", np.hstack([vis_verts,vis_verts]))
    time_ts = time.time()

vis.destroy_window()
    # 3 space = 0.059 m



