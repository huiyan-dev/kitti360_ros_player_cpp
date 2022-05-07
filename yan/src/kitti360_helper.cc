// ROS
#include <ros/ros.h>
// for kitti360 data publish
#include "kitti360_publisher.hpp"
#include "yan_utility.hpp"

using namespace std;

int main(int argc, char **argv){
  ros::init(argc, argv, "kitti360_helper");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  ros::Rate loop_rate(10);

  
  std::string dataset_root, sequence;
  // read param
  // 读取参数
  pnh.getParam("dataset_root", dataset_root);
  pnh.getParam("sequence", sequence);
  ROS_INFO("%s", ("Use dataset : " + dataset_root + sequence).c_str());

  // read timestamp.txt
  // kitti360 
  stringstream timestamp_file_path;
  timestamp_file_path << dataset_root << "data_3d_raw/" << sequence << "velodyne_points/timestamps.txt";
  ifstream timestamp_file(timestamp_file_path.str(), ios::in);

  yan::Kitti360StereoImagePub kitti360_img_pub(nh, pnh, dataset_root, sequence);
  yan::Kitti360Velodyne64LidarPub kitti360_velodyne64_pub(nh, pnh, dataset_root, sequence);
  yan::Kitti360GroudTruthPub kitti360_gt_pub(nh, pnh, dataset_root, sequence);

  string line;
  size_t frame_idx = 0;
  double_t timestamp;
  yan::DateToTimestamp<double_t> dttt;
  while(getline(timestamp_file, line) && ros::ok){  
    
    std_msgs::Header header;
    header.stamp = ros::Time().fromSec(dttt(line));
    header.frame_id = "/world";
    
    // if(!kitti360_img_pub.publish(header, frame_idx)){
    //   ROS_INFO("System error : please check log.");
    // }
    // if(!kitti360_velodyne64_pub.publish(header, frame_idx)){
    //   ROS_INFO("System error : please check log.");
    // }
    kitti360_img_pub.publish(header, frame_idx);
    kitti360_velodyne64_pub.publish(header, frame_idx);
    kitti360_gt_pub.publish(header, frame_idx);
    frame_idx++;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}