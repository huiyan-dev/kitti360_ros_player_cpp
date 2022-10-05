//
// Created by huiyan on 10/5/22.
//
#pragma once

// for ROS image message
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
// kitti360 dataset parameters
#include "common/config.h"

namespace kitti360 {
namespace fs = boost::filesystem;

class DataImuRawPub : public DataRawPub {
 public:
  DataImuRawPub();
  ~DataImuRawPub() override = default;
  void Publish(rosbag::Bag& bag) final;
  // clang-format off
  double GetCurrentTimestamp() final { return imu_timestamps_[current_frame_index];}
  // clang-format on

 private:
  std::vector<double> imu_timestamps_;
};
class Data2DRawPub : public DataRawPub {
 public:
  Data2DRawPub();
  ~Data2DRawPub() override = default;
  void Publish(rosbag::Bag& bag) override;
  // clang-format off
  double GetCurrentTimestamp() final { return left_perspective_timestamps_[current_frame_index];}
  // clang-format on

 private:
  std::vector<double> left_perspective_timestamps_;
  std::vector<double> right_perspective_timestamps_;
  std::vector<fs::path> left_perspective_img_filenames;
  std::vector<fs::path> right_perspective_img_filenames;
  image_transport::Publisher left_perspective_pub_;
  image_transport::Publisher right_perspective_pub_;
};
class Data3DRawPub : public DataRawPub {
 public:
  Data3DRawPub();
  ~Data3DRawPub() override = default;
  void Publish(rosbag::Bag& bag) override;
  // clang-format off
  double GetCurrentTimestamp() final { return lidar_velo_timestamps_[current_frame_index];}
  // clang-format on

 private:
  std::vector<double> lidar_velo_timestamps_;// velodyne
  std::vector<fs::path> lidar_velo_filenames;
  ros::Publisher lidar_velo_pub_;
};

}// namespace kitti360