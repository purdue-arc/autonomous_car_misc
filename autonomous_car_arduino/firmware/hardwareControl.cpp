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

const byte MOTOR_FEEDBACK_PIN = 3;
const byte RECIEVER_DRIVE_PIN = 4;
const byte MOTOR_SIGNAL_PIN = 5;
const byte SERVO_SIGNAL_PIN = 6;
const byte RECIEVER_STEER_PIN = 13;
const byte SERVO_FEEDBACK_PIN = 14;

#define BUFFER_SIZE 10

float getBufferAverage(float * buffer)
{
  float averageEffort = 0;
  cli();
  for (byte i = 0; i < BUFFER_SIZE; i++)
  {
    averageEffort += *(buffer + i);
  }
  sei();
  averageEffort /= BUFFER_SIZE;
  return averageEffort;
}

void commandCallback(const autonomous_car_arduino_msgs::ArduinoCommand& msg_command)
{
  // Do something
}

void populateReport(autonomous_car_arduino_msgs::ArduinoReport& msg_report)
{
  // Populate Message
  msg_report.sensor_feedback.steering_angle = analogRead(SERVO_FEEDBACK_PIN);
  msg_report.sensor_feedback.speed = getBufferAverage(&motor_feedback_buffer);
  msg_report.radio_command.steering_angle = getBufferAverage(&steer_effort_buffer);
  msg_report.radio_command.speed = getBufferAverage(&drive_effort_buffer);
  msg_report.drive_battery_voltage = 0.0;
  msg_report.computer_battery_voltage = 0.0;
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
volatile float motor_feedback_buffer [BUFFER_SIZE] = {};
volatile byte motor_buffer_index;

// Reading input from RC controller
volatile unsigned long last_steer_edge_us;
volatile float steer_effort_buffer [BUFFER_SIZE] = {};
volatile byte steer_buffer_index;

volatile unsigned long last_drive_edge_us;
volatile float drive_effort_buffer [BUFFER_SIZE] = {};
volatile byte drive_buffer_index;

void setup()
{
  last_report_ms = 0;
  last_spin_ms = 0;

  nh.initNode();
  nh.advertise(reporter);
  nh.subscribe(commander);

  pinMode(MOTOR_FEEDBACK_PIN, INPUT;
  pinMode(RECIEVER_DRIVE_PIN, INPUT);
  pinMode(MOTOR_SIGNAL_PIN, OUTPUT);
  pinMode(SERVO_SIGNAL_PIN, OUTPUT);
  pinMode(RECIEVER_STEER_PIN, INPUT);
  pinMode(SERVO_FEEDBACK_PIN, INPUT)

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
    populateReport(msg_report);
    reporter.publish(&msg_report);
  }
  if(millis() >= last_spin_ms + 1)
  {
    nh.spinOnce();
  }
}

// Port B callback, PB5, pin 13, reciever steer
ISR(PCINT0_vect)
{
  // Get the time. Resolution = 8 us
  unsigned long time = micros();
  if(digitalRead(RECIEVER_STEER_PIN) == LOW)
  {
    // This is the falling edge, calculate effort
    float effort = (time - (last_steer_edge_us + 500)) / 500.0;
    current_steer_effort[steer_buffer_index] = effort;
    steer_buffer_index++;
    if (steer_buffer_index >= BUFFER_SIZE)
    {
      steer_buffer_index = 0;
    }
  }
  else
  {
    // This is the rising edge, record the time
    last_drive_edge_us = time;
  }
}

// Port D callback, PD4, pin 4, reciever drive
ISR(PCINT2_vect)
{
  // Get the time. Resolution = 8 us
  unsigned long time = micros();
  if(digitalRead(RECIEVER_DRIVE_PIN) == LOW)
  {
    // This is the falling edge, calculate effort
    float effort = (time - (last_drive_edge_us + 500)) / 500.0;
    current_drive_effort[drive_buffer_index] = effort;
    drive_buffer_index++;
    if (drive_buffer_index >= BUFFER_SIZE)
    {
      drive_buffer_index = 0;
    }
  }
  else
  {
    // This is the rising edge, record the time
    last_drive_edge_us = time;
  }
}
