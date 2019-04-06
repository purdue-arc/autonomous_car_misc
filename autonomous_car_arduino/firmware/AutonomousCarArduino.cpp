#include <ros.h>
#include <autonomous_car_arduino_msgs/ArduinoCommand.h>
#include <autonomous_car_arduino_msgs/ArduinoReport.h>
#include "HardwareControl.h"

/*
  Connections:
  Motor Feedback  ->  pin 3   PD3   INT1
  Reciever Drive  ->  pin 4   PD4
  Motor Signal    ->  pin 5   PD5   PWM
  Servo Signal    ->  pin 6   PD6   PWM
  Servo Feedback  ->  pin 14  PC0   A0
  Reciever Steer  ->  pin 13  PB5
*/

// ROS stuff
ros::NodeHandle nh;
ros::Subscriber commandSubscriber;
ros::Publisher reportPublisher;
ros::Timer reportTimer;

// We can just use the same message instance forever
autonomous_car_arduino_msgs::ArduinoReport msg_report;

// Link to hardware
HardwareControl hardware();

int main()
{
  nh.initNode();

  reportPublisher = nh.advertise<autonomous_car_arduino_msgs::ArduinoReport>("hardwareReport", 1);
  reportTimer = nh.createTimer(ros::Duration(0.01), reportTimerCallback);
  commandSubscriber = nh.subscribe("hardwareCommand", 1, commandCallback);

  ros.spin();
  return 0;
}

void reportTimerCallback(const ros::TimerEvent& e)
{
  msg_report.sensor_feedback.steering_angle = hardware.getSteeringFeedback();
  msg_report.sensor_feedback.speed = hardware.getDriveFeedback();
  msg_report.radio_control.steering_angle = hardware.getDriveRadioControl();
  msg_report.radio_control.speed = hardware.getSteeringRadioControl();
  reportPublisher.publish(&msg_report);
}

void commandCallback(const autonomous_car_arduino_msgs::ArduinoCommand& msg_command)
{
  // Set the efforts
  hardware.setSteeringEffort(msg_command.autonomous_control.steering_angle);
  hardware.setDriveEffort(msg_command.autonomous_control.drive);
}
