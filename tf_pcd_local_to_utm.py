#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 30 14:29:34 2023

@author: mzaffar
"""
# conda environment to use 'localizationpriusdev'
import utm
import copy
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

directory='/media/mzaffar/extradata/codes/autoware_iv/ws_lidarslam/'
pcd_filename = "map_front_2024_03_20-17_05_56.pcd"

"""
INPUTS NEEDED:

PCD map: map in local coordinate system that is output by SLAM
MGRS coordinates for the first frame in rosbag
rot_world_to_imu_quat: quaternion rotation from the IMU for the first frame in rosbag
rot_imu_to_bl_quat: reading from the tf_static between frame_id sensor_kit_base_link and child_frame oxts_link
rot_lidar_to_bl_quat: reading from the tf_static between frame_id sensor_kit_base_link and LiDAR (Ouster or robosense)

*To help you double-check that you are looking at the correct fields, I have kept the values from the rosbag 2024_03_20-17_05_56 as default values in this code.

OUTPUT:

A globally registered pointcloud in .pcd and .ply formats
"""
rot_world_to_imu_quat = R.from_quat([0.3907894027172118, 0.5930803027552981, -0.5874990235995148, -0.38779413930231776]) # newtestlocation reading from the IMU
rot_imu_to_bl_quat = R.from_quat([-0.7069644353521388,0.0017681153831721886,0.0017674147704959862,0.7072446798387757]) # newtestlocation reading from the tf_static between frame_id sensor_kit_base_link and child_frame oxts_link 
rot_lidar_to_bl_quat = R.from_quat([0.004999213564421915,8.749516935828507e-05,0.017498888046131707,0.9998343808478892]) # newtestlocation reading from the tf_static between frame_id sensor_kit_base_link and LIDAR (IIRC both Ouster and robosense have the same rotation)

tf_imu_to_world_4x4 = np.asarray([[1.0,  0,  0,  0], [0,  1.0,  0, 0], [0, 0,  1.0, 0], [0,  0,  0.0,  1.0]])
tf_imu_to_world_4x4[0:3, 0:3] = np.linalg.inv(rot_world_to_imu_quat.as_matrix())

tf_imu_to_world_4x4 =  np.linalg.inv(tf_imu_to_world_4x4)

tf_imu_to_world_4x4[0,3] = 94084.56
tf_imu_to_world_4x4[1,3] = 61943.91


rot_bl_to_lidar_quat = R.from_quat(R.from_matrix(np.linalg.inv(rot_lidar_to_bl_quat.as_matrix())).as_quat())

correction_rot = R.from_matrix([[-1,0,0], [0,-1,0], [0,0,1]]).as_matrix()
rot_imu_to_lidar_quat = R.from_quat(R.from_matrix(correction_rot @ rot_bl_to_lidar_quat.as_matrix() @ rot_imu_to_bl_quat.as_matrix()).as_quat())

# rot_imu_to_lidar_quat = R.from_quat([-0.49960183664463353, 0.49999984146591747, 0.49999984146591747, 0.5003981633553666]) # reading from the tf_static between frame_id sensor_kit_base_link and child_frame oxts_link 


tf_imu_to_lidar_4x4 = np.asarray([[1.0,  0,  0,  0], [0,  1.0,  0, 0], [0, 0,  1.0, 0], [0,  0,  0.0,  1.0]])
tf_imu_to_lidar_4x4[0:3, 0:3] = rot_imu_to_lidar_quat.as_matrix()

tf_lidar_to_imu_4x4 = np.linalg.inv(tf_imu_to_lidar_4x4)

tf_lidar_to_world = tf_imu_to_world_4x4 @ tf_lidar_to_imu_4x4

print(tf_lidar_to_world)

def transform_points(mat, points):
    """Apply 4x4 homogenous coordinate transform to Nx3 points."""
    return mat[:3, :3].dot(points.T).T + mat[:3, 3][None]

if __name__ == "__main__":
    # Load saved point cloud and visualize it
    pcd_load = o3d.io.read_point_cloud(directory+pcd_filename) #"map_noimu_backend_rosbag2_2023_12_11-10_44_05.pcd"
    # o3d.visualization.draw_geometries([pcd_load])

    # convert Open3D.o3d.geometry.PointCloud to numpy array
    xyz_load = np.asarray(pcd_load.points)
    
    xyz_temp = xyz_load.copy()
    remove_GP = False
    
    if (remove_GP == True):
	    # mask = xyz_temp[:,2] < 25.0
	    mask = np.logical_and(xyz_temp[:,2] < 16.0 , xyz_temp[:,2] > -5)
	    xyz_temp = xyz_temp[mask,:]
    
    xyz_new = transform_points(tf_lidar_to_world, xyz_temp)

    print(xyz_new[int(len(xyz_new)/2)])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz_new)

    o3d.io.write_point_cloud(directory+"map_front_2024_03_20-17_05_56_globallyregistered_withGP.pcd", pcd) 
    o3d.io.write_point_cloud(directory+"map_front_2024_03_20-17_05_56_globallyregistered_withGP.ply", pcd) 

    o3d.visualization.draw_geometries([pcd])
    
