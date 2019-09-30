#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "MPU6050.h"

class RosIMU
{
public:
  RosIMU();
  ~RosIMU();
private:
  void publishIMU(const ros::TimerEvent& e);

  ros::NodeHandle m_nh;
  ros::Timer m_imuTimer;
  ros::Publisher m_imuPublisher;

  MPU6050 m_imuHandle;
};
