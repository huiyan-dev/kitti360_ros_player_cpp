// ROS
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Header.h>
// for kitti360 data publish
#include "kitti360.h"

using namespace std;

int main(int argc, char **argv){
  ros::init(argc, argv, "kitti360_helper");
  ros::NodeHandle nh;
  ros::Rate loop_rate(100);

  kitti360::DataImuRawPub data_imu_raw_pub;
  kitti360::Data2DRawPub data_2d_raw_pub;
  kitti360::Data3DRawPub data_3d_raw_pub;

  rosbag::Bag kitti360_bag;

  size_t frame_idx = 0;
  double cur_time;
  std_msgs::Header header;
  header.frame_id = data_imu_raw_pub.frame_id_world;

  while(ros::ok){
    cur_time = data_imu_raw_pub.GetCurrentTimestamp();
    header.stamp = ros::Time().fromSec(cur_time);
    // Just publish it use imu timestamp as reference.
    data_imu_raw_pub.Publish(kitti360_bag);
    // image perspective and oxts (sync with cam0 at 10 Hz).
    if(cur_time >= data_2d_raw_pub.GetCurrentTimestamp()) {
      data_2d_raw_pub.Publish(kitti360_bag);
    }
    // point cloud
    if(cur_time >= data_3d_raw_pub.GetCurrentTimestamp()) {
      data_3d_raw_pub.Publish(kitti360_bag);
    }
    frame_idx++;
    loop_rate.sleep();
  }
  return 0;
}