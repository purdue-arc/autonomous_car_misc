#include <ros.h>
#include <autonomous_car_arduino_msgs/ArduinoCommand.h>
#include <autonomous_car_arduino_msgs/ArduinoReport.h>

#include <Arduino.h>
#include <avr/interrupt.h>

/*
  Connections:
  Motor Feedback  ->  pin 3   PD3   INT1
  Motor Control   ->  pin 4   PD4
  Motor Output    ->  pin 5   PD5   PWM
  Servo Output    ->  pin 6   PD6   PWM
  Servo Feedback  ->  pin 14  PC0   A0
  Servo Control   ->  pin 13  PB5
*/

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

void updateMotorSpeed()
{
  // I have no idea
}

// volatile unsigned long last_motor_edge_us;
// volatile float current_motor_effort;

// ROS stuff
ros::NodeHandle nh;
autonomous_car_arduino_msgs::ArduinoReport msg_report;
ros::Subscriber<autonomous_car_arduino_msgs::ArduinoCommand> commander("hardwareCommand", commandCallback);
ros::Publisher reporter("hardwareReport", &msg_report);

// Delay between sending status messages
const byte report_delay_ms = 10;
unsigned long last_report_ms;
unsigned long last_spin_ms;

volatile unsigned long last_motor_edge_us;
volatile float current_motor_effort;

volatile unsigned long last_steer_edge_us;
volatile bool last_steer_state;
volatile float current_steer_effort;

volatile unsigned long last_drive_edge_us;
volatile bool last_drive_state;
volatile float current_drive_effort;

void setup()
{
  last_report_ms = 0;
  last_spin_ms = 0;
  nh.initNode();
  nh.advertise(reporter);
  nh.subscribe(commander);

  attachInterrupt(1, updateMotorSpeed, FALLING);

  // Set up the interrupts
  cli();
  PCICR |= 1 << 0;  // turn on port B
  PCICR |= 1 << 2;  // Turn on port D
  PCMSK0 |= (1 << 6);   // Turn on PB5
  PCMSK2 |= (1 << 5);   // Turn on PD4
  sei();
}

void loop()
{
  if(millis() >= last_report_ms + report_delay_ms)
  {
    last_report_ms = millis();
    populateReport(&msg_report);
    reporter.publish(&msg_report);
  }
  if(millis() >= last_spin_ms + 1)
  {
    nh.spinOnce();
  }
}

// Port B callback, PB5, pin 13, servo control
void ISR(PCINT0_vect)
{
  // Get the time
  unsigned long time = micros();
  if(last_steer_state)
  {
    // This is the falling edge, calculate effort
    current_steer_effort = (time - (last_steer_edge_us + 500)) / 500.0;
  }
  else
  {
    last_drive_edge_us = time;
  }
  last_steer_state = !last_steer_state;
}

// Port D callback, PD4, pin 4, motor control
void ISR(PCINT2_vect)
{
  // Get the time
  unsigned long time = micros();
  if(last_drive_state)
  {
    // This is the falling edge, calculate effort
    current_drive_effort = (time - (last_drive_edge_us + 500)) / 500.0;
  }
  else
  {
    last_drive_edge_us = time;
  }
  last_drive_state = !last_drive_state;
}
