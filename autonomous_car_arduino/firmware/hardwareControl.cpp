#include <ros.h>
#include <autonomous_car_arduino_msgs/ArduinoCommand.h>
#include <autonomous_car_arduino_msgs/ArduinoReport.h>

#include <Arduino.h>

void commandCallback(const autonomous_car_arduino_msgs::ArduinoCommand& msg_command)
{
  // Do something
}

void populateReport(autonomous_car_arduino_msgs::ArduinoReport *msg_report)
{
  // Do something
  msg_report->drive_battery_voltage = 0.0;
  msg_report->computer_battery_voltage = 0.0;
}

// ROS stuff
ros::NodeHandle nh;
autonomous_car_arduino_msgs::ArduinoReport msg_report;
ros::Subscriber<autonomous_car_arduino_msgs::ArduinoCommand> commander("hardwareCommand", commandCallback);
ros::Publisher reporter("hardwareReport", &msg_report);

// Delay between sending status messages
const byte report_delay_ms = 10;
unsigned long last_report_ms;

void setup()
{
  last_report_ms = 0;
  nh.initNode();
  nh.advertise(reporter);
  nh.subscribe(commander);
}

void loop()
{
  if(millis() >= last_report_ms + report_delay_ms)
  {
    last_report_ms = millis();
    populateReport(&msg_report);
    reporter.publish(&msg_report);
  }
  nh.spinOnce();
  delay(1);
}
