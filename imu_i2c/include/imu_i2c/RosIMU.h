#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "MPU6050.h"

class RosIMU
{
public:
  RosIMU();
  ~RosIMU();
private:
  publishIMU(const ros::TimerEvent& e);

  ros::NodeHandle nh;
  ros::Timer imuTimer;
  ros::Publisher imuPublisher;

  MPU6050 imuHandle;
};
