#include <ros/ros.h>
#include "RosIMU.h"

int main(int argc, char **argv)
{
  RosIMU ros_imu = RosIMU;

  //ros::Rate loop_rate(200);

  while(ros::ok())
  {
    ros::spinOnce();
  }
}
