//
// Created by huiyan on 10/5/22.
//
#include "kitti360.h"

#include <glog/logging.h>

#include "common/time_helper.h"

namespace kitti360 {

DateToTimestamp<double> dttt;

void inline ReadTimestamps(const std::string& file_name, std::vector<double>& timestamps) {
  std::ifstream in_file(file_name);
  std::string line;
  while (getline(in_file, line)) {
    timestamps.emplace_back(dttt(line));
  }
}
DataImuRawPub::DataImuRawPub() {
  current_frame_index = 0;
  // Read timestamps
  std::string sequence_dir = home_dir + data_root_dir + data_imu_raw_dir + sequence_extract + "oxts/";
  ReadTimestamps(sequence_dir + "timestamps.txt", timestamps_);
}

void DataImuRawPub::Publish(rosbag::Bag& bag) {
//  LOG(INFO) << "DataImuRawPub : " << GetCurrentTimestamp() << " , " << current_frame_index_;
  current_frame_index++;
}
inline double DataImuRawPub::GetCurrentTimestamp() {
  return timestamps_[current_frame_index];
}

Data2DRawPub::Data2DRawPub() {
  current_frame_index = 0;
  std::string sequence_dir = home_dir + data_root_dir + data_2d_raw_dir + sequence_sync;
  ReadTimestamps(sequence_dir + "image_00/timestamps.txt", left_perspective_timestamps_);
  ReadTimestamps(sequence_dir + "image_01/timestamps.txt", right_perspective_timestamps_);
  assert(left_perspective_timestamps_.size() == right_perspective_timestamps_.size());
}

void Data2DRawPub::Publish(rosbag::Bag& bag) {
//  LOG(INFO) << "Data2DRawPub : " << GetCurrentTimestamp() << " , " << current_frame_index_;
  current_frame_index++;
}
inline double Data2DRawPub::GetCurrentTimestamp() {
  return left_perspective_timestamps_[current_frame_index]; // cam0 as image reference
}

Data3DRawPub::Data3DRawPub() {
  current_frame_index = 0;
  std::string sequence_dir = home_dir + data_root_dir + data_3d_raw_dir + sequence_sync;
  ReadTimestamps(sequence_dir + "velodyne_points/timestamps.txt", velo_timestamps_);
}

void Data3DRawPub::Publish(rosbag::Bag& bag) {
//  LOG(INFO) << "Data3DRawPub : " << GetCurrentTimestamp() << " , " << current_frame_index_;
  current_frame_index++;
}
inline double Data3DRawPub::GetCurrentTimestamp() {
  return velo_timestamps_[current_frame_index];
}
}// end namespace kitti360