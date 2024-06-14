## What is this repo for?
Given a pointcloud map created using a SLAM method, the code in this repository help you to register that pointcloud to global MGRS coordinates and visualize it in Open3D.

## Assumptions
It is assumed that you have a rosbag for the area you want to create the map for, and that you have a local map created for this environment using a [SLAM method](https://github.com/rsasaki0109/lidarslam_ros2).

The linked SLAM method has its stand-alone documentation and I found it sufficient. Some tips are below:

 - The SLAM method expects two topics `/input_cloud
   (sensor_msgs/PointCloud2)` and `/tf_static`. So you will have to
   [remap](https://answers.ros.org/question/345960/rosbag2-remap-topic/) your pointcloud topic when replaying the rosbag.
   
 - When playing a rosbag to create the map, only publish the `/input_cloud` and the `/tf_static` topic. If you mistakenly publish the /tf topic, then there will be two nodes (rosbag play and lidar_slam method) disagreeing on /tf. You can re-record the rosbag with only these two topics, and then play this re-recorded rosbag. See 3.1 [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html).

**INPUTS NEEDED:**
The following inputs are needed.
 - PCD map: a map in local coordinate system that is output by SLAM 
 - MGRS coordinates for the first frame in rosbag (read from GPS topic)
  - rot_world_to_imu_quat: quaternion rotation from the IMU for the first frame in rosbag
  - rot_imu_to_bl_quat: reading from the tf_static between frame_id
   sensor_kit_base_link and child_frame oxts_link 
 - rot_lidar_to_bl_quat: reading from the tf_static between frame_id sensor_kit_base_link and LiDAR (Ouster or robosense)

*To help you double-check that you are looking at the correct fields, I have kept the values from the `rosbag 2024_03_20-17_05_56` as default values in this code.

**OUTPUT:**
A globally registered pointcloud in `.pcd` and `.ply` formats

## How to use?
There is an accompanying conda `environment.yaml` file, please create and activate this environment.

After activating this environment, please update the inputs (outlined above) in `tf_pcd_local_to_utm.py` and simply run it with Python. All inputs other than the local PCD map, you can [read](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html) from the rosbag. 


