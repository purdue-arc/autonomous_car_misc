#include "RosIMU.h"

RosIMU::RosIMU() :
  m_nh(),
  m_imuTimer(),
  m_imuPublisher(),
  m_imuHandle(8, 0x68)
{
  m_imuTimer = m_nh.createTimer(ros::Duration(0.01), &RosIMU::publishIMU, this);
  m_imuPublisher = m_nh.advertise<sensor_msgs::Imu>("imu", 10);
}

RosIMU::~RosIMU() = default;

void RosIMU::publishIMU(const ros::TimerEvent& e)
{
  const MPU6050::imu_data& data = m_imuHandle.readIMU();

  sensor_msgs::Imu imu_message;

  imu_message.header.stamp = e.current_real;
  imu_message.header.frame_id = "imu";

  imu_message.angular_velocity.x = data.gyro_x_radps;
  imu_message.angular_velocity.y = data.gyro_y_radps;
  imu_message.angular_velocity.z = data.gyro_z_radps;

  imu_message.linear_acceleration.x = data.accelerometer_x_mps;
  imu_message.linear_acceleration.y = data.accelerometer_y_mps;
  imu_message.linear_acceleration.z = data.accelerometer_z_mps;

  m_imuPublisher.publish(imu_message);
}
