//
// Created by huiyan on 10/5/22.
//

// data publish
#include "kitti360.h"
// for handle timestamp
#include "common/time_helper.h"
// for log
#include <glog/logging.h>
// for image publish
#include <opencv2/opencv.hpp>

namespace kitti360 {

DateToTimestamp<double> dttt;

inline void ReadTimestamps(const std::string& file_name, std::vector<double>& timestamps) {
  std::ifstream in_file(file_name);
  std::string line;
  while (getline(in_file, line)) {
    timestamps.emplace_back(dttt(line));
  }
}
inline void ReadFileNamesByDirectory(const std::string& directory, std::string extension, std::vector<fs::path>& paths) {
  if(!fs::is_directory(directory)) {
    LOG(ERROR) << directory << " is not a directory";
  }
  for(auto &path : fs::directory_iterator(directory)) {
    if(path.path().extension() == extension) {
      paths.emplace_back(path.path());
    }
  }
  std::sort(paths.begin(), paths.end());
}
DataImuRawPub::DataImuRawPub() {
  current_frame_index = 0;
  // Read timestamps
  std::string sequence_dir = home_dir + data_root_dir + data_imu_raw_dir + sequence_extract + "oxts/";
  ReadTimestamps(sequence_dir + "timestamps.txt", imu_timestamps_);
}

void DataImuRawPub::Publish(rosbag::Bag& bag) {
  std::string left_perspective_img_file_names;
//  LOG(INFO) << "DataImuRawPub : " << GetCurrentTimestamp() << " , " << current_frame_index_;
  current_frame_index++;
}
inline double DataImuRawPub::GetCurrentTimestamp() {
  return imu_timestamps_[current_frame_index];
}

Data2DRawPub::Data2DRawPub() {
  current_frame_index = 0;
  std::string sequence_dir = home_dir + data_root_dir + data_2d_raw_dir + sequence_sync;
  ReadTimestamps(sequence_dir + "image_00/timestamps.txt", left_perspective_timestamps_);
  ReadTimestamps(sequence_dir + "image_01/timestamps.txt", right_perspective_timestamps_);
  assert(left_perspective_timestamps_.size() == right_perspective_timestamps_.size());
  ReadFileNamesByDirectory(sequence_dir + "image_00/data_rect/", data_2d_raw_img_extension, left_perspective_img_file_names);
  ReadFileNamesByDirectory(sequence_dir + "image_01/data_rect/", data_2d_raw_img_extension, right_perspective_img_file_names);
  assert(left_perspective_img_file_names.size() == right_perspective_img_file_names.size());
  if(left_perspective_timestamps_.size() != left_perspective_img_file_names.size()) {
    LOG(ERROR) << "Missing timestamp or data_2d_raw, please check!";
  }

  image_transport::ImageTransport img_tp(nh);
  left_perspective_pub_ = img_tp.advertise(topic_name_left_perspective, 2);
  right_perspective_pub_ = img_tp.advertise(topic_name_right_perspective, 2);
}

void Data2DRawPub::Publish(rosbag::Bag& bag) {
  cv::Mat left_perspective_img = cv::imread(left_perspective_img_file_names[current_frame_index].string(), cv::IMREAD_GRAYSCALE);
  cv::Mat right_perspective_img = cv::imread(right_perspective_img_file_names[current_frame_index].string(), cv::IMREAD_GRAYSCALE);

  std_msgs::Header header;
  header.stamp = ros::Time().fromSec(GetCurrentTimestamp()); // use left image as reference
  header.frame_id = frame_id_cam0;

  sensor_msgs::ImageConstPtr left_perspective_msg_cptr = cv_bridge::CvImage(header, "mono8", left_perspective_img).toImageMsg();
  sensor_msgs::ImageConstPtr right_perspective_msg_cptr = cv_bridge::CvImage(header, "mono8", right_perspective_img).toImageMsg();
  left_perspective_pub_.publish(left_perspective_msg_cptr);
  right_perspective_pub_.publish(right_perspective_msg_cptr);
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