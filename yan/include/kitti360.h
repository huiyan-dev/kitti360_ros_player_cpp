//
// Created by huiyan on 10/5/22.
//
#pragma once

#include <rosbag/bag.h>

#include "common/config.h"

namespace kitti360 {

class DataImuRawPub : public DataRawPub {
 public:
  DataImuRawPub();
  ~DataImuRawPub() override = default;
  void Publish(rosbag::Bag& bag) final;
  double GetCurrentTimestamp() final;

 private:
  std::vector<double> timestamps_;
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