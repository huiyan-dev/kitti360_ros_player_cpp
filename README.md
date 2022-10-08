# A package for KITTI-360 Dataset preprocessing

This package provide a cpp tool for KITTI-360 dataset. It especially for SLAM task which load sensors
data of Velodyne, Perspective Cameras and Raw IMU Data (OXTS 3003 100Hz).

# Dependencies

Test in ubuntu 20.04 ros noetic

[ROS install documentation](https://www.ros.org/blog/getting-started/)

Boost c++
```shell
sudo apt install libboost-all-dev
```

## Optional
[imu tools in rviz](https://wiki.ros.org/imu_tools)
```shell
sudo apt-get install ros-noetic-imu-tools
```



# Build
Git clone the repository to your catkin workspace. Then run catkin_make:
```shell
cd ~/catkin_ws
catkin_make
```
# Datasets Settings
It divides the dataset follow the [official documentation](https://www.cvlibs.net/datasets/kitti-360/documentation.php).
Mutable configurations is the top 2 level path in the dataset root directory of KITTI-360 as below:

    ├── KITTI-360 (Dataset root directory)
    
        ├── calibration (calibration files)
    
        ├── data_2d_raw (2d raw data)
    
        ├── data_3d_raw (3d raw data)
    
        ├── data_poses (sync poses with cam0 at 10 Hz)
    
        ├── data_poses_oxts_extract (raw oxts measurements at 100 Hz)

# Example
For run a visualization in rviz you can run:
```shell
roslaunch kitti360_ros_player_cpp kitti360_helper.launch
```
Default sequence is "2013_05_28_drive_0003_sync"

You can change ${PROJECT_ROOT_DIR}/config/kitti360.yaml for your purpose.