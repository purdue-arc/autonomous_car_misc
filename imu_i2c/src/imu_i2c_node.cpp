#include <ros/ros.h>
#include <imu_i2c/RosIMU.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "i2c_imu_node");
  RosIMU ros_imu = RosIMU();
  ros::spin();

  return 0;
}
