#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import open3d as o3d

mask = False
if __name__ == "__main__":
    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud("map_noimu_backend_rosbag2_2023_12_11-10_44_05_globallyregistered_noGP_noreflc_nodynnobjs.pcd")
    print(pcd)
    xyz = np.asarray(pcd.points)

    if (mask):
        mask = np.logical_and(xyz[:,2] < 15.0 , xyz[:,2] > 1.0)
        print(mask.shape)
        print(xyz.shape)
        xyz = xyz[mask,:]
        print(xyz.shape)

    pcd.points = o3d.utility.Vector3dVector(xyz)
    o3d.visualization.draw_geometries([pcd])
    print("Downsample the point cloud with a voxel of 0.01")
    downpcd = pcd.voxel_down_sample(voxel_size=0.01)
    o3d.visualization.draw_geometries([downpcd])
    
    # o3d.io.write_point_cloud("map_noimu_backend_rosbag2_2023_12_11-10_44_05_globallyregistered_noGP_noreflc_nodynnobjs.pcd", pcd)
