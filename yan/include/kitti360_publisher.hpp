// c++
#include <string>
#include <sstream>
#include <fstream>
// for file io
// for ROS image message
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
// for image of cv::Mat to ROS sensor_msgs::Image 
#include <cv_bridge/cv_bridge.h>
// for read image
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
// for read lidar .bin file and publish
#include <open3d_conversions/open3d_conversions.h>
#include <open3d/Open3D.h>
#include <sensor_msgs/PointCloud2.h>
// tools for handle data
#include "yan_utility.hpp"
// ros nav_msgs
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
namespace yan{
  // These classes can use abstract class to rebuild structure

  /**
   * @brief kitti360 stereo image data loader and publish it to ros
   *        kitti360 双目图像数据加载, 同时提供发布到ROS的接口
   * 
   */
  class Kitti360StereoImagePub{
  private:
    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;

    image_transport::Publisher left_pub_;
    image_transport::Publisher right_pub_;
    std::string dataset_root_;
    std::string sequence_;
  
  public:
    /**
     * @brief Construct a new Kitti360 Stereo Image Pub object
     * 
     * @param nh  name start at "/"
     * @param pnh name start at "~/"  --->  "/node_name/"
     */
    Kitti360StereoImagePub(ros::NodeHandle &nh, ros::NodeHandle &pnh) 
                : nh_(nh), pnh_(pnh) {
      
      // initrialize ros publisher of left/righr image
      // 初始化两个 ros publisher, 对应左右目的图像
      image_transport::ImageTransport img_tp(pnh);
      left_pub_ = img_tp.advertise("left_image", 2);
      right_pub_ = img_tp.advertise("right_image", 2);
      
    }
    /**
     * @brief Construct a new Kitti360 Stereo Image Pub object
     * 
     * @param nh name start at "/"
     * @param pnh  name start at "~/"  --->  "/node_name/"
     * @param dataset_root directory of dataset
     * @param sequence sequence directory of dataset
     */
    Kitti360StereoImagePub(ros::NodeHandle &nh, ros::NodeHandle &pnh,
                           std::string &dataset_root, std::string &sequence) 
                : nh_(nh), pnh_(pnh),
                  dataset_root_(dataset_root), sequence_(sequence)  {
      
      // initrialize ros publisher of left/righr image
      // 初始化两个 ros publisher, 对应左右目的图像
      image_transport::ImageTransport img_tp(pnh);
      left_pub_ = img_tp.advertise("left_raw_image", 2);
      right_pub_ = img_tp.advertise("right_raw_image", 2);
      
    }

    ~Kitti360StereoImagePub(){}

    /**
     * @brief 发布消息
     * 
     * @param header std_msgs::Header
     * @param frame_index frame的索引
     * @return true if message pub sucess / 消息发布成功返回true
     * @return false vice vsersa  / 反之亦然
     */
    bool publish(std_msgs::Header &header, size_t &frame_index){
      // dataset directory must be not null
      // 数据集的路径不能为空
      if(dataset_root_.empty() || sequence_.empty()){
        ROS_INFO("Please check your dataset directory or sequence name !");
        return false;
      }

      // read data, you can see KITTI360 dataset format in http://www.cvlibs.net/datasets/kitti-360/documentation.php
      // 读取数据, 可以从下面的链接了解 KITTI360 数据集的格式 http://www.cvlibs.net/datasets/kitti-360/documentation.php
      std::stringstream left_img_root_ss, right_img_root_ss, suffix;
      suffix << std::setfill('0') << std::setw(10) << frame_index << ".png";
      left_img_root_ss << dataset_root_ << "data_2d_raw/" << sequence_ << "image_00/data_rect/" << suffix.str();
      right_img_root_ss << dataset_root_ << "data_2d_raw/" << sequence_ << "image_01/data_rect/"  << suffix.str();

      cv::Mat left_img = cv::imread(left_img_root_ss.str(), cv::IMREAD_GRAYSCALE);
      cv::Mat right_img = cv::imread(right_img_root_ss.str(), cv::IMREAD_GRAYSCALE);

      sensor_msgs::ImageConstPtr rosmsg_left_img_cptr_ 
                = cv_bridge::CvImage(header, "mono8", left_img).toImageMsg();
      sensor_msgs::ImageConstPtr rosmsg_right_img_cptr_
                = cv_bridge::CvImage(header, "mono8", right_img).toImageMsg();

      // ROS_INFO("Publish images : %s" , left_img_root_ss.str().c_str());
      left_pub_.publish(rosmsg_left_img_cptr_);
      // ROS_INFO("Publish images : %s" , right_img_root_ss.str().c_str());
      right_pub_.publish(rosmsg_right_img_cptr_);

      return true;
    }

    void set_data_dir(std::string &dataset_root, std::string &sequence){
      set_dataset_root(dataset_root);
      set_sequence(sequence);
    }

    std::string dataset_root() const { return dataset_root_; }
    void set_dataset_root(const std::string &dataset_root) { dataset_root_ = dataset_root; }

    std::string sequence() const { return sequence_; }
    void set_sequence(const std::string &sequence) { sequence_ = sequence; }
  
  };

  class Kitti360Velodyne64LidarPub{
  private:
    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;
    std::string dataset_root_;
    std::string sequence_;
    ros::Publisher laser_pub_;
  public:

    ~Kitti360Velodyne64LidarPub(){}

    Kitti360Velodyne64LidarPub(ros::NodeHandle &nh, ros::NodeHandle &pnh)
          : nh_(nh), pnh_(pnh) {
      laser_pub_ = pnh.advertise<sensor_msgs::PointCloud2>("velodyne_raw_points", 2);
    }
    Kitti360Velodyne64LidarPub(ros::NodeHandle &nh, ros::NodeHandle &pnh,
                               std::string &dataset_root, std::string &sequence)
          : nh_(nh), pnh_(pnh), dataset_root_(dataset_root), sequence_(sequence) {
      laser_pub_ = pnh.advertise<sensor_msgs::PointCloud2>("velodyne_raw_points", 2);
    }
    /**
     * @brief publish lidar raw points / 发布lidar原始点云
     * 
     * @param header std_msgs::Header
     * @param frame_index frame的索引
     * @return true if message pub sucess / 消息发布成功返回true
     * @return false vice vsersa  / 反之亦然
     */
    bool publish(std_msgs::Header &header, size_t &frame_index){
      // dataset directory must be not null
      // 数据集的路径不能为空
      if(dataset_root_.empty() || sequence_.empty()){
        ROS_INFO("Please check your dataset directory or sequence name!");
        return false;
      }

      // read lidar .bin data, convert to std::vector<double>
      std::stringstream bin_path_ss;
      bin_path_ss << dataset_root_ << "data_3d_raw/" << sequence_ << "velodyne_points/data/" 
               << std::setfill('0') << std::setw(10) << frame_index << ".bin";
      std::string tempp = bin_path_ss.str();
      std::vector<float> lidar_data_buffer = yan::read_lidar_data<float>(bin_path_ss.str());
      const size_t num_elements = lidar_data_buffer.size();
      // build open3d::geometry::PointCloud by lidar_data_buffer 
      open3d::geometry::PointCloud laser_cloud;
      for(int i = 0; i < num_elements; i += 4){
        laser_cloud.points_.emplace_back(
          Eigen::Vector3d(
            lidar_data_buffer[i], lidar_data_buffer[i+1], lidar_data_buffer[i+2]
          )
        );
        // due to open3d not support rgbi (intensity), use colors to save intensity value
        // it means that laser_cloud.colors_[0] respond for point intensity
        laser_cloud.colors_.emplace_back(
          Eigen::Vector3d(
            lidar_data_buffer[i+3], 0.0, 0.0
          )
        );
      }
      sensor_msgs::PointCloud2 laser_cloud_msg;
      open3d_conversions::open3dToRos(laser_cloud, laser_cloud_msg);
      laser_cloud_msg.header = header;

      // ROS_INFO("Publish lidar points : %s", bin_path_ss.str().c_str());
      laser_pub_.publish(laser_cloud_msg);
      return true;
    }
    
    std::string dataset_root() const { return dataset_root_; }
    void set_dataset_root(const std::string &dataset_root) { dataset_root_ = dataset_root; }

    std::string sequence() const { return sequence_; }
    void set_sequence(const std::string &sequence) { sequence_ = sequence; }
    
  };
  class Kitti360SICKLidarPub{
    
  };
  class Kitti360GroudTruthPub{

  private:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;
    std::string dataset_root_;
    std::string sequence_;

    // calib_cam_to_pose_[i] : cam{i} to gps/imu coordinate
    // cam0 and cam1 is perspective, and cam2 and cam3 is fisheye camera.
    std::vector<Eigen::Isometry3d,
           Eigen::aligned_allocator<Eigen::Isometry3d>> calib_cam_to_pose_;
    Eigen::Isometry3d calib_cam_to_velo_;
    Eigen::Isometry3d calib_sick_to_velo_;
    // calib_cam_intrinsics_[i] : cam{i} intrinsics
    Eigen::Isometry3d calib_cam_intrinsics_;
    // camera coordinate and lidar coordinate are different 
    // see http://www.cvlibs.net/datasets/kitti-360/documentation.php
    Eigen::Quaterniond q_transform_;
    typedef std::pair<int, Eigen::Isometry3d> pose_cache_type_;
    pose_cache_type_ pose_cache_;
    // The world origin of Kitti360 Datasets is in the center of all sequences
    // I use fisrt frame pose in current sequence as new world origin as all things done in a single sequence
    Eigen::Isometry3d init_pose_;
    // !!!  frame index may not continuous, sometime movement is too smaller threshold  !!!!
    // use pose_cache_ as current pose
    int cur_frame_id_ = -1;
    // Each line has 17 numbers, the first number is an integer denoting the frame index.
    // The rest is a 4x4 matrix denoting the rigid body transform from the rectified perspective
    // camera coordinates to the world coordinate system.
    std::string cam0_to_world_path_;
    std::ifstream cam0_to_world_file_;
    // gps / imu coordinate to world : Each line has 13 numbers, the first number is an integer 
    // denoting the frame index. The rest is a 3x4 matrix denoting the rigid body transform from 
    // GPU/IMU coordinates to a world coordinate system.
    std::string imu_to_world_path_;
    std::ifstream imu_to_world_file_;
    std::string oxts_root_path_;

    ros::Publisher gt_odom_pub_;
    ros::Publisher gt_path_pub_;
    nav_msgs::Path global_pathGT_;
    nav_msgs::Odometry odomGT_;

  public:

    ~Kitti360GroudTruthPub(){}

    Kitti360GroudTruthPub(ros::NodeHandle &nh, ros::NodeHandle &pnh)
          : nh_(nh), pnh_(pnh) {
      gt_odom_pub_ = nh.advertise<nav_msgs::Odometry>("odometry_gt", 10);
      gt_path_pub_ = nh.advertise<nav_msgs::Path>("path_gt", 10);
    }


    Kitti360GroudTruthPub(ros::NodeHandle &nh, ros::NodeHandle &pnh,
                               std::string &dataset_root, std::string &sequence)
          : nh_(nh), pnh_(pnh), dataset_root_(dataset_root), sequence_(sequence) {
      gt_odom_pub_ = nh.advertise<nav_msgs::Odometry>("odometry_gt", 10);
      gt_path_pub_ = nh.advertise<nav_msgs::Path>("path_gt", 10);
      // 读取 calib_cam_to_velo_
      std::string cam_to_velo_path = dataset_root + "calibration/calib_cam_to_velo.txt";
      std::ifstream calib_cam_to_velo_file_(cam_to_velo_path, std::ifstream::in);
      read_calib_file<Eigen::Isometry3d>(calib_cam_to_velo_file_, calib_cam_to_velo_, 3, 4);
      // cam0 to world(kitti360 world origin is the center of all sequences)
      cam0_to_world_path_ = dataset_root + "data_poses/" + sequence + "cam0_to_world.txt";
      cam0_to_world_file_ = move(std::ifstream(cam0_to_world_path_, std::ifstream::in));
      // cam0 to imu/gps
      imu_to_world_path_ = dataset_root + "data_poses/" + sequence + "cam0_to_pose.txt";
      imu_to_world_file_ = move(std::ifstream(imu_to_world_path_, std::ifstream::in));
      // 读取第一个位姿, 因为kitti360里位姿不一定是连续的, 有些时刻运动量太小而被忽略
      read_pose_file<Eigen::Isometry3d, pose_cache_type_>(cam0_to_world_file_, pose_cache_, 4, 4);
      // pose_cache_.second is cam_to_world, convert to velodyne
      init_pose_ = pose_cache_.second;

      // TODO: imu 数据暂时还未处理
      oxts_root_path_ = dataset_root + "data_poses/" + sequence + "oxts/";

      // select velodyne coordinate as world coordinate
      odomGT_.header.frame_id = "world";
      global_pathGT_.header.frame_id = "world";

      Eigen::Matrix3d R_transform;
      R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0;
      q_transform_ = Eigen::Quaterniond(R_transform);
    }
    bool publish(std_msgs::Header &header, size_t &frame_index){
      std::string line, tmp;
      odomGT_.header.stamp = header.stamp;

      if(pose_cache_.first < frame_index) {
        read_pose_file<Eigen::Isometry3d, pose_cache_type_>(cam0_to_world_file_, pose_cache_, 4, 4);
      }
      // change world coordinate
      // pose_cache_.second = init_pose_.inverse() * pose_cache_.second;
      // note : world origin has move to the center of velodyne frame 0
      // velo ----> world(frame 0 center)
      Eigen::Quaterniond q_w_i(pose_cache_.second.rotation());
      Eigen::Quaterniond q_w_velo = q_transform_ * Eigen::Quaterniond(init_pose_.rotation().inverse()) * q_w_i;
      // q_w_cam0 = Eigen::Quaterniond(init_pose_.data()).inverse() * q_w_cam0;
      Eigen::Vector3d t(q_transform_ * Eigen::Quaterniond(init_pose_.rotation().inverse()) * (pose_cache_.second.translation() - init_pose_.translation()));

      odomGT_.pose.pose.orientation.x = q_w_velo.x();
      odomGT_.pose.pose.orientation.y = q_w_velo.y();
      odomGT_.pose.pose.orientation.z = q_w_velo.z();
      odomGT_.pose.pose.orientation.w = q_w_velo.w();
      odomGT_.pose.pose.position.x = t(0);
      odomGT_.pose.pose.position.y = t(1);
      odomGT_.pose.pose.position.z = t(2);
      gt_odom_pub_.publish(odomGT_);

      geometry_msgs::PoseStamped poseGT;
      poseGT.header.stamp = odomGT_.header.stamp;
      poseGT.pose = odomGT_.pose.pose;
      global_pathGT_.header.stamp = header.stamp;
      global_pathGT_.poses.push_back(poseGT);
      gt_path_pub_.publish(global_pathGT_);
    }

  };
}