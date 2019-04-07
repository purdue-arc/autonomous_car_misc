#include "RosIMU.h"

RosIMU::RosIMU() :
  imuHandle(7)
{
  ros::NodeHandle nh;
  imuPublisher = nh.advertise<sensor_msgs::Imu>("imu", 1000);
  ros::Timer imuTimer = nh.createTimer(ros::Duration(0.01), &RosIMU::publishIMU, this);
}

RosIMU::~RosIMU() = default;

void RosIMU::publishIMU(const ros::TimerEvent& e)
{
  const MPU6050::imu_data& data = imuHandle.readIMU();

  sensor_msgs::Imu imu_message;
  imu_message.angular_velocity.x = data.accelerometer_x_mps;
  imu_message.angular_velocity.y = data.accelerometer_y_mps;
  imu_message.angular_velocity.z = data.accelerometer_z_mps;

  imu_message.linear_acceleration.x = data.gyro_x_radps;
  imu_message.linear_acceleration.y = data.gyro_y_radps;
  imu_message.linear_acceleration.z = data.gyro_z_radps;

  imuPublisher.publish(imu_message);
}
