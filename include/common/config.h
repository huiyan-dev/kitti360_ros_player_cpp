//
// Created by huiyan on 10/5/22.
//

#pragma once
// for log
#include <glog/logging.h>
// ros
#include <ros/ros.h>
#include <rosbag/bag.h>
// std
#include <string>

namespace kitti360 {

struct Config {
  ros::NodeHandle nh;

  std::string home_dir{};
  std::string data_root_dir{"/Datasets/KITTI-360/"};
  std::string data_imu_raw_dir{"data_poses_oxts_extract/"};
  std::string data_2d_raw_dir{"data_2d_raw/"};
  std::string data_3d_raw_dir{"data_3d_raw/"};
  std::string data_calibration_dir{"calibration/"};
  std::string data_poses_gt_dir{"data_poses/"};
  std::string data_2d_raw_img_extension{".png"};
  std::string data_3d_raw_lidar_extension{".bin"};

  std::string sequence_number{"0003"};
  std::string sequence_sync;
  std::string sequence_extract;

  std::string topic_name_imu{"kitti360/data_imu_raw/data_imu_raw"};
  std::string topic_name_left_perspective{"kitti360/data_2d_raw/left_perspective"};
  std::string topic_name_right_perspective{"kitti360/data_2d_raw/right_perspective"};
  std::string topic_name_lidar_velo{"kitti360/data_3d_raw/lidar_velo"};
  std::string topic_name_gt_path{"kitti360/data_poses/gt_path"};
  std::string topic_name_gt_odom{"kitti360/data_poses/gt_odom"};

  std::string frame_id_world{"world"};
  std::string frame_id_lidar{"lidar"};
  std::string frame_id_cam0{"cam0"};
  std::string frame_id_imu{"imu"};

  std::string data_output_dir{"output/"};

  Config() {
    home_dir = std::getenv("HOME");
    nh.getParam("kitti360/data_root_dir", data_root_dir);
    nh.getParam("kitti360/data_poses_oxts_extract_dir", data_imu_raw_dir); // device is oxts 3003
    nh.getParam("kitti360/data_2d_raw_dir", data_2d_raw_dir);
    nh.getParam("kitti360/data_3d_raw_dir", data_3d_raw_dir);
    nh.getParam("kitti360/data_calibration_dir", data_calibration_dir);
    nh.getParam("kitti360/data_poses_gt_dir", data_poses_gt_dir);
    nh.getParam("kitti360/data_2d_raw_img_extension", data_2d_raw_img_extension);
    nh.getParam("kitti360/data_3d_raw_lidar_extension", data_3d_raw_lidar_extension);

    nh.getParam("kitti360/sequence_number", sequence_number);
    sequence_sync = "2013_05_28_drive_" + sequence_number + "_sync/";
    sequence_extract = "2013_05_28_drive_" + sequence_number + "_extract/";

    nh.getParam("kitti360/topic_name_imu", topic_name_imu);
    nh.getParam("kitti360/topic_name_left_perspective", topic_name_left_perspective);
    nh.getParam("kitti360/topic_name_right_perspective", topic_name_right_perspective);
    nh.getParam("kitti360/topic_name_lidar_velo", topic_name_lidar_velo);
    nh.getParam("kitti360/topic_name_gt_path", topic_name_gt_path);
    nh.getParam("kitti360/topic_name_gt_odom", topic_name_gt_odom);

    nh.getParam("kitti360/data_output_dir", data_output_dir);
    if((home_dir.empty() && data_root_dir.empty() && data_imu_raw_dir.empty())) {  // use imu as reference
      LOG(ERROR) << "Missing data, please check!";
    }
  }

};

class DataRawPub : public Config {
 public:
  DataRawPub() = default;
  virtual ~DataRawPub() = default;
  virtual void Publish() = 0;
  virtual double GetCurrentTimestamp() = 0;

  size_t current_frame_index{};
  std::string frame_id_name{};
};


}// namespace kitti360
