//
// Created by huiyan on 10/5/22.
//
#pragma once

// for ROS image message
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
// kitti360 dataset parameters
#include "common/config.h"
// for pose representation
#include <Eigen/Geometry>
// for data_poses which are not continuous
#include <unordered_map>

namespace kitti360 {
// for file read and iteration
namespace fs = boost::filesystem;
// for poses of ground truth
template<typename T>
using PoseType = Eigen::Transform<T, 3, Eigen::Isometry>;
template<typename T>
using PoseTypeVector = std::vector<PoseType<T>, Eigen::aligned_allocator<PoseType<T>>>;
template<typename T>
using PoseTypeUnorderedMap =
                std::unordered_map<size_t, PoseType<T>,
                std::hash<size_t>, std::equal_to<size_t>,
                Eigen::aligned_allocator<std::pair<const size_t, PoseType<T>>>>;

struct DataCalibrationRaw : public Config {
 public:
  DataCalibrationRaw();
  void PublishStaticTransform();
  // camXtoImu[i] : camX_to_imu, X = 0, 1, 2, 3
  PoseTypeVector<double> calib_camX_to_imu;
  PoseType<double> calib_cam0_to_velo;
  PoseType<double> calib_sick_to_velo;
  PoseType<double> calib_P_rect_00;
  PoseType<double> calib_P_rect_01;
  PoseType<double> calib_R_rect_00;
  PoseType<double> calib_R_rect_01;
};

class DataImuRawPub : public DataRawPub {
 public:
  DataImuRawPub();
  ~DataImuRawPub() override = default;
  void Publish() final;
  // clang-format off
  double GetCurrentTimestamp() final { return imu_timestamps_[current_frame_index];}
  // clang-format on

 private:
  std::vector<double> imu_timestamps_;
  std::vector<fs::path> imu_filenames;// oxts 3003
  ros::Publisher imu_pub_;
};
class Data2DRawPub : public DataRawPub {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Data2DRawPub();
  ~Data2DRawPub() override = default;
  void Publish() override;
  void OdometryCallBack(const nav_msgs::Odometry::ConstPtr &odom);
  // clang-format off
  // inline functions : getters
  double GetCurrentTimestamp() final { return left_perspective_timestamps_[current_frame_index];}
  const PoseType<double>& getCurrentPoseCam0ToWorld() const {return data_poses_cam0_to_world_gt_[current_frame_index];}
  const PoseType<double>& getCurrentPoseImuToWorld() const {return data_poses_imu_to_world_gt_[current_frame_index];}
  // inline functions : setters
  // clang-format on

 private:

  std::vector<double> left_perspective_timestamps_;
  std::vector<double> right_perspective_timestamps_;
  std::vector<fs::path> left_perspective_img_filenames;
  std::vector<fs::path> right_perspective_img_filenames;
  image_transport::Publisher left_perspective_pub_;
  image_transport::Publisher right_perspective_pub_;
  // Publish the ground truth here because the oxts poses sync at 10 Hz with camera 0 (left perspective)
  // The world is the center of all sequences
  // Here use cam0_to_world as ground truth
  // The data_poses_* is not continuous(but just a few frames).
  PoseTypeVector<double> data_poses_cam0_to_world_gt_;
  PoseTypeVector<double> data_poses_imu_to_world_gt_;
  // for RVIZ show
  ros::Publisher gt_path_pub_;
  ros::Publisher gt_odom_pub_;
  ros::Subscriber gt_odom_sub_;

  nav_msgs::Path gt_path_world_;
  nav_msgs::Odometry gt_odom_world_;
};
class Data3DRawPub : public DataRawPub {
 public:
  Data3DRawPub();
  ~Data3DRawPub() override = default;
  void Publish() override;
  // clang-format off
  double GetCurrentTimestamp() final { return lidar_velo_timestamps_[current_frame_index];}
  // clang-format on
 private:
  std::vector<double> lidar_velo_timestamps_;// velodyne
  std::vector<fs::path> lidar_velo_filenames;
  ros::Publisher lidar_velo_pub_;
};

}// namespace kitti360