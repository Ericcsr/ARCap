import open3d as o3d
from argparse import ArgumentParser


parser = ArgumentParser()
parser.add_argument('--pcd1', type=int, required=True)
parser.add_argument('--pcd2', type=int, required=True)
args = parser.parse_args()


pcd1 = o3d.io.read_point_cloud(f'cropped_DepthMap{args.pcd1}.bin.ply')
pcd2 = o3d.io.read_point_cloud(f'cropped_DepthMap{args.pcd2}.bin.ply')

pcd1.paint_uniform_color([1, 0, 0])
pcd2.paint_uniform_color([0, 1, 0])

o3d.visualization.draw_geometries([pcd1, pcd2])

