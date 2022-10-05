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
// kitti360 dataset parameters
#include "common/config.h"

namespace kitti360 {
namespace fs = boost::filesystem;

class DataImuRawPub : public DataRawPub {
 public:
  DataImuRawPub();
  ~DataImuRawPub() override = default;
  void Publish(rosbag::Bag& bag) final;
  double GetCurrentTimestamp() final;

 private:
  std::vector<double> imu_timestamps_;
};
class Data2DRawPub : public DataRawPub {
 public:
  Data2DRawPub();
  ~Data2DRawPub() override = default;
  void Publish(rosbag::Bag& bag) override;
  double GetCurrentTimestamp() override;

 private:
  std::vector<double> left_perspective_timestamps_;
  std::vector<double> right_perspective_timestamps_;
  std::vector<fs::path> left_perspective_img_file_names;
  std::vector<fs::path> right_perspective_img_file_names;
  image_transport::Publisher left_perspective_pub_;
  image_transport::Publisher right_perspective_pub_;
};
class Data3DRawPub : public DataRawPub {
 public:
  Data3DRawPub();
  ~Data3DRawPub() override = default;
  void Publish(rosbag::Bag& bag) override;
  double GetCurrentTimestamp() override;

 private:
  std::vector<double> velo_timestamps_;// velodyne
};

}// namespace kitti360