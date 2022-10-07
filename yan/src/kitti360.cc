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
// ros message and tools
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace kitti360 {

DateToTimestamp<double> dttt;

inline void ReadTimestamps(const std::string& filename, std::vector<double>& timestamps) {
  std::ifstream in_file(filename);
  std::string line;
  while (getline(in_file, line)) {
    timestamps.emplace_back(dttt(line));
  }
}

inline void ReadFileNamesByDirectory(const std::string& directory, const std::string& extension, std::vector<fs::path>& paths) {
  if (!fs::is_directory(directory)) {
    LOG(ERROR) << directory << " is not a directory";
  }
  for (auto& path : fs::directory_iterator(directory)) {
    if (path.path().extension() == extension) {
      paths.emplace_back(path.path());
    }
  }
  std::sort(paths.begin(), paths.end());
}

void FillDataToRosMsgByBinFile(const std::string& filename, const std::string& extension, sensor_msgs::PointCloud2& msg) {
  // Just fill the msg.data and related fields by the .bin data
  std::ifstream data_file(filename, std::ifstream::in | std::ifstream::binary);
  data_file.seekg(0, std::ios::end);
  const size_t num_floats = data_file.tellg() / sizeof(float);
  data_file.seekg(0, std::ios::beg);
  msg.width = num_floats / 4;
  msg.height = 1;
  msg.point_step = 4 * sizeof(float);
  msg.row_step = 4 * sizeof(float) * msg.width;
  msg.is_dense = true;
  msg.data.resize(num_floats * sizeof(float));
  // note : unsigned long --> long
  data_file.read(reinterpret_cast<char*>(&msg.data[0]), static_cast<long>(sizeof(float) * num_floats));
  // add fields
  std::vector<std::string> name_fields{"x", "y", "z", "intensity"};
  uint32_t offset = 0;
  for (const auto& name_field : name_fields) {
    sensor_msgs::PointField field;
    field.name = name_field;
    field.datatype = 7;// float32
    field.count = 1;
    field.offset = offset;
    offset += 4;
    msg.fields.push_back(field);
  }
}

void ReadImuDataFromFile(const std::string& filename, sensor_msgs::Imu& msg) {
  // Just read the Imu data
  std::ifstream infile(filename);
  std::string line, temp;
  std::stringstream ss;
  double ax, ay, az, wx, wy, wz, roll, pitch, yaw;
  auto stream_to_null = [&](int n) -> void {
    std::string null_str;
    for (int i = 0; i < n; ++i) ss >> null_str;
  };
  getline(infile, line);
  ss = std::stringstream(line);
  // lat, lon, alt
  stream_to_null(3);
  // roll : range: -pi   .. +pi
  // pitch : range: -pi/2 .. +pi/2
  // yaw : range: -pi   .. +pi
  ss >> roll >> pitch >> yaw;
  // vn, ve
  stream_to_null(2);
  // vf:    forward velocity, i.e. parallel to earth-surface (m/s)
  // vl:    leftward velocity, i.e. parallel to earth-surface (m/s)
  // vu:    upward velocity, i.e. perpendicular to earth-surface (m/s)
  stream_to_null(3);
  // ax:    acceleration in x, i.e. in direction of vehicle front (m/s^2)
  // ay:    acceleration in y, i.e. in direction of vehicle left (m/s^2)
  // az:    acceleration in z, i.e. in direction of vehicle top (m/s^2)
  ss >> ax >> ay >> az;
  // af, al, au
  stream_to_null(3);
  // wx:    angular rate around x (rad/s)
  // wy:    angular rate around y (rad/s)
  // wz:    angular rate around z (rad/s)
  ss >> wx >> wy >> wz;
  // wf, wl, wu, pos_accuracy, vel_accuracy, navstat, numsats, posmode, velmode, orimode
  stream_to_null(10);
  msg.linear_acceleration.x = ax;
  msg.linear_acceleration.y = ay;
  msg.linear_acceleration.z = az;
  msg.angular_velocity.x = wx;
  msg.angular_velocity.y = wy;
  msg.angular_velocity.z = wz;
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();
}

template<typename T>
PoseType<T> InterpolatePoseType(const T &t, const PoseType<T> &p1, const PoseType<T> &p2) {
  PoseType<T> ret = PoseType<T>::Identity();
  Eigen::Quaternion<T> q1; q1 = p1.rotation();
  Eigen::Quaternion<T> q2; q2 = p2.rotation();
  Eigen::Quaternion<T> q3 = q1.template slerp(t, q2);
  ret.template prerotate(q3);
  ret.template pretranslate((1 - t) * p1.translation() + t * p2.translation());
  return ret;
}

template <typename T>
void ReadPosesByFileName(const std::string &filename, const int&rows, const int&cols, PoseTypeVector<T> &poses) {
  if(poses.empty()) {
    LOG(WARNING) << "Poses is empty, please check.";
  }
  std::ifstream infile(filename);
  size_t id, cnt = 0;
  PoseType<T> pose;
  std::stringstream ss;
  std::string line;
  std::vector<bool> is_pose_available(poses.size(), false);
  while(getline(infile, line)) {
    ss = std::stringstream(line);
    ss >> id;
    pose = PoseType<T>::Identity();
    for(int i = 0; i < rows; ++i) {
      for(int j = 0; j < cols; ++j) {
        ss >> pose(i, j);
      }
    }
    poses[id] = pose;
    is_pose_available[id] = true;
    cnt++;
  }
  if(cnt == poses.size()) return;
  // ensure the left has a pose value
  size_t t = 0;
  while(!is_pose_available[t]) t++;
  while(t > 0) {
    poses[t - 1] = poses[t];
    is_pose_available[t - 1] = true;
    t--;
  };
  // do the interpolation
  for(size_t i = 0; i < poses.size(); ++i) {
    if(!is_pose_available[i]) {
      size_t left = i - 1, right = i + 1;
      while(right < poses.size() && !is_pose_available[right]) right++;
      if(right < poses.size()) {
        // Just use the frame num as interpolate rate
        while(i < right) {
          T rate = (T)((long double)(i - left) / (right - left));
          poses[i] = InterpolatePoseType<T>(rate, poses[left], poses[right]);
          i++;
        }
      } else {
        while(left < poses.size() - 1) {
          poses[left + 1] = poses[left];
          left++;
        }
        return ;
      }
    }
  }
}

template <typename T>
void ReadPosesByFileNameAndField(const std::string &filename, const std::string &field, const int&rows, const int&cols, PoseType<T> &pose) {
  std::ifstream infile(filename);
  pose = PoseType<T>::Identity();
  std::stringstream ss;
  std::string line, temp;
  auto read_pose = [&]() ->void {
    for(int i = 0; i < rows; ++i) {
      for(int j = 0; j < cols; ++j) {
        ss >> pose(i, j);
      }
    }
  };
  if(field.empty()) {
    // if not get a filed, just read the value as pose
    getline(infile, line);
    ss = std::stringstream(line);
    read_pose();
    return;
  }
  // otherwise there is a filed in the beginning of the input line
  while(getline(infile, line)) {
    ss = std::stringstream(line);
    ss >> temp;
    if(temp == field) {
      read_pose();
      return;
    }
  }
}

DataCalibrationRaw::DataCalibrationRaw() {
  std::string calibration_dir = home_dir + data_root_dir + data_calibration_dir;
  calib_camX_to_imu.resize(4);
  // DataCalibrationRaw files are short lines, it cost few time.
  for(int i = 0; i < 4; ++i) {
    ReadPosesByFileNameAndField(calibration_dir + "calib_cam_to_pose.txt", "image_0" + std::to_string(i) + ":", 3, 4, calib_camX_to_imu[i]);
  }
  ReadPosesByFileNameAndField(calibration_dir + "calib_cam_to_velo.txt", "", 3, 4, calib_cam0_to_velo);
  ReadPosesByFileNameAndField(calibration_dir + "calib_sick_to_velo.txt", "", 3, 4, calib_sick_to_velo);
  ReadPosesByFileNameAndField(calibration_dir + "perspective.txt", "P_rect_00:", 3, 4, calib_P_rect_00);
  ReadPosesByFileNameAndField(calibration_dir + "perspective.txt", "P_rect_01:", 3, 4, calib_P_rect_01);
  ReadPosesByFileNameAndField(calibration_dir + "perspective.txt", "R_rect_00:", 3, 3, calib_R_rect_00);
  ReadPosesByFileNameAndField(calibration_dir + "perspective.txt", "R_rect_01:", 3, 3, calib_R_rect_01);
}

// Coordinate axis : cam0 , IMU , Veledyne
// cam 0:        Z           IMU:       X           Velodyne and World:   Z     X
//              ^                      ^                                  ^    ^
//             /                      /                                   |   /
//            /                      /                                    |  /
//           *---------> X          *---------> Y                         | /
//           |                      |                         Y <---------*
//           |                      |
//           |                      |
//           v  Y                   v Z
// eg : imu -> cam0   Eigen::Matrix3d Axis_cam0_imu{
//                                     {0, 1, 0},
//                                     {1, 0, 0},
//                                     {0, 0, 1}};
// All the poses in the kitti360 files has already been transformed by the coordinate transform.
void DataCalibrationRaw::PublishStaticTransform() {
  static tf2_ros::StaticTransformBroadcaster static_br;

  geometry_msgs::TransformStamped T_cam0_imu_ros = tf2::eigenToTransform(calib_camX_to_imu[0].inverse());
  T_cam0_imu_ros.header.frame_id = frame_id_cam0;
  T_cam0_imu_ros.child_frame_id = frame_id_imu;
  static_br.sendTransform(T_cam0_imu_ros);

  geometry_msgs::TransformStamped T_cam0_lidar_ros = tf2::eigenToTransform(calib_cam0_to_velo.inverse());
  T_cam0_lidar_ros.header.frame_id = frame_id_cam0;
  T_cam0_lidar_ros.child_frame_id = frame_id_lidar;
  static_br.sendTransform(T_cam0_lidar_ros);
}

DataImuRawPub::DataImuRawPub() {
  current_frame_index = 0;
  // Read timestamps
  std::string sequence_dir = home_dir + data_root_dir + data_imu_raw_dir + sequence_extract + "oxts/";
  ReadTimestamps(sequence_dir + "timestamps.txt", imu_timestamps_);
  ReadFileNamesByDirectory(sequence_dir + "data/", ".txt", imu_filenames);

  imu_pub_ = nh.advertise<sensor_msgs::Imu>(topic_name_imu, 50);
}

void DataImuRawPub::Publish() {
  std::string left_perspective_img_file_names;
  sensor_msgs::Imu data_imu_current;
  data_imu_current.header.frame_id = frame_id_imu;
  data_imu_current.header.stamp = ros::Time().fromSec(imu_timestamps_[current_frame_index]);
  ReadImuDataFromFile(imu_filenames[current_frame_index].string(), data_imu_current);
  imu_pub_.publish(data_imu_current);
  current_frame_index++;
}

Data2DRawPub::Data2DRawPub() {
  current_frame_index = 0;
  std::string sequence_dir = home_dir + data_root_dir + data_2d_raw_dir + sequence_sync;
  ReadTimestamps(sequence_dir + "image_00/timestamps.txt", left_perspective_timestamps_);
  ReadTimestamps(sequence_dir + "image_01/timestamps.txt", right_perspective_timestamps_);
  assert(left_perspective_timestamps_.size() == right_perspective_timestamps_.size());
  ReadFileNamesByDirectory(sequence_dir + "image_00/data_rect/", data_2d_raw_img_extension, left_perspective_img_filenames);
  ReadFileNamesByDirectory(sequence_dir + "image_01/data_rect/", data_2d_raw_img_extension, right_perspective_img_filenames);
  assert(left_perspective_img_filenames.size() == right_perspective_img_filenames.size());
  if (left_perspective_timestamps_.size() != left_perspective_img_filenames.size()) {
    LOG(ERROR) << "Missing timestamp or data_2d_raw, please check!";
  }

  image_transport::ImageTransport img_tp(nh);
  left_perspective_pub_ = img_tp.advertise(topic_name_left_perspective, 2);
  right_perspective_pub_ = img_tp.advertise(topic_name_right_perspective, 2);

  std::string data_poses_sequence_dir = home_dir + data_root_dir + data_poses_gt_dir + sequence_sync;
  // It will interpolate while kitti360 gt poses is not continuous
  // cam0_to_world : id + 4x4 matrix
  data_poses_cam0_to_world_gt_.resize(left_perspective_img_filenames.size());
  ReadPosesByFileName(data_poses_sequence_dir + "cam0_to_world.txt", 4, 4, data_poses_cam0_to_world_gt_);
  // imu_to_world : id + 3x4 matrix
  data_poses_imu_to_world_gt_.resize(left_perspective_img_filenames.size());
  ReadPosesByFileName(data_poses_sequence_dir + "poses.txt", 3, 4, data_poses_imu_to_world_gt_);

  gt_path_pub_ = nh.advertise<nav_msgs::Path>(topic_name_gt_path, 10);
  gt_odom_pub_ = nh.advertise<nav_msgs::Odometry>(topic_name_gt_odom, 10);
  gt_path_world_.header.frame_id = frame_id_world;
  gt_odom_world_.header.frame_id = frame_id_world;
}

void Data2DRawPub::Publish() {
  std_msgs::Header header;
  header.stamp = ros::Time().fromSec(GetCurrentTimestamp());// use left image as reference
  header.frame_id = frame_id_cam0;

  cv::Mat left_perspective_img = cv::imread(left_perspective_img_filenames[current_frame_index].string(), cv::IMREAD_GRAYSCALE);
  cv::Mat right_perspective_img = cv::imread(right_perspective_img_filenames[current_frame_index].string(), cv::IMREAD_GRAYSCALE);
  sensor_msgs::ImageConstPtr left_perspective_msg_cptr = cv_bridge::CvImage(header, "mono8", left_perspective_img).toImageMsg();
  sensor_msgs::ImageConstPtr right_perspective_msg_cptr = cv_bridge::CvImage(header, "mono8", right_perspective_img).toImageMsg();
  left_perspective_pub_.publish(left_perspective_msg_cptr);
  right_perspective_pub_.publish(right_perspective_msg_cptr);

  auto cur_pose = getCurrentPoseCam0ToWorld();
  // gt poses : cam0 to world
  geometry_msgs::PoseStamped pose_stamp;
  pose_stamp.header.frame_id = frame_id_world;
  pose_stamp.header.stamp = header.stamp;
  pose_stamp.pose = tf2::toMsg(cur_pose);
  gt_path_world_.poses.push_back(pose_stamp);
  gt_path_pub_.publish(gt_path_world_);
  // gt odom : cam0 to world
  gt_odom_world_.header.frame_id = frame_id_world;
  gt_odom_world_.header.stamp = header.stamp;
  gt_odom_world_.pose.pose = tf2::toMsg(cur_pose);
  gt_odom_pub_.publish(gt_odom_world_);
  // tf2 : for RVIZ
  // Point_target = T_target_source * Point_source
  // T_target_source is a transform from source to target
  // ros and eigen in the tail is the type of variable
  // Point_target is a point in target coordinate ...
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped T_world_cam0_ros = tf2::eigenToTransform(cur_pose);
  T_world_cam0_ros.header.frame_id = frame_id_world;
  T_world_cam0_ros.header.stamp = header.stamp;
  T_world_cam0_ros.child_frame_id = frame_id_cam0;
  br.sendTransform(T_world_cam0_ros);

  current_frame_index++;
}

Data3DRawPub::Data3DRawPub() {
  current_frame_index = 0;
  std::string sequence_dir = home_dir + data_root_dir + data_3d_raw_dir + sequence_sync;
  ReadTimestamps(sequence_dir + "velodyne_points/timestamps.txt", lidar_velo_timestamps_);
  ReadFileNamesByDirectory(sequence_dir + "velodyne_points/data", data_3d_raw_lidar_extension, lidar_velo_filenames);
  assert(lidar_velo_timestamps_.size() == lidar_velo_filenames.size());
  if (lidar_velo_timestamps_.size() != lidar_velo_filenames.size()) {
    LOG(ERROR) << "Missing lidar data or timestamp, please check!";
  }
  lidar_velo_pub_ = nh.advertise<sensor_msgs::PointCloud2>(topic_name_lidar_velo, 2);
}

void Data3DRawPub::Publish() {
  sensor_msgs::PointCloud2 lidar_velo_msg;
  lidar_velo_msg.header.frame_id = frame_id_lidar;
  lidar_velo_msg.header.stamp = ros::Time().fromSec(lidar_velo_timestamps_[current_frame_index]);
  FillDataToRosMsgByBinFile(lidar_velo_filenames[current_frame_index].string(), data_3d_raw_lidar_extension, lidar_velo_msg);
  lidar_velo_pub_.publish(lidar_velo_msg);
  current_frame_index++;
}

}// end namespace kitti360