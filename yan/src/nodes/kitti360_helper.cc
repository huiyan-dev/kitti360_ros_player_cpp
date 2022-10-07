// ROS
#include <ros/ros.h>
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
  kitti360::DataCalibrationRaw data_calibration_raw_pub;
  data_calibration_raw_pub.PublishStaticTransform();

  double cur_time;
  // note loop_rate is 100 Hz which is imu frequency as the time resolution.
  while(ros::ok){
    cur_time = data_imu_raw_pub.GetCurrentTimestamp();
    // Just publish it use imu timestamp as reference.
    data_imu_raw_pub.Publish();
    // image perspective and oxts (sync with cam0 at 10 Hz).
    if(cur_time >= data_2d_raw_pub.GetCurrentTimestamp()) {
      data_2d_raw_pub.Publish();
    }
    // point cloud
    if(cur_time >= data_3d_raw_pub.GetCurrentTimestamp()) {
      data_3d_raw_pub.Publish();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}