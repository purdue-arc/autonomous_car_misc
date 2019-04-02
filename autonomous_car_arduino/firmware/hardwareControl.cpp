#include <ros.h>
#include <autonomous_car_arduino_msgs/ArduinoCommand.h>
#include <autonomous_car_arduino_msgs/ArduinoReport.h>

#include <Arduino.h>

ros::NodeHandle nh;
ros::Publisher reportPub;
ros::Subscriber commandSub;

// Delay between sending status messages
const byte report_delay_ms = 10;
unsigned long last_report_ms;

void setup()
{
  last_report_ms = 0;
  nh.initNode();
  reportPub = nh.advertise<autonomous_car_arduino_msgs::ArduinoReport>("hardwareReport", 100);
  commandSub = nh.subscribe("hardwareCommand", 1, &commandCallback)
  nh.advertise(chatter);
}

void loop()
{
  if(millis() >= last_report_ms + report_delay_ms)
  {
    last_report_ms = millis();
    msg_report = autonomous_car_arduino_msgs::ArduinoReport.empty();
    reportSub.publish(&msg_report);
  }
  nh.spinOnce();
  delay(1);
}

void commandCallback(const sutonomous_car_arduino_msgs::ArduinoReport& msg)
{
  // Do something
}
