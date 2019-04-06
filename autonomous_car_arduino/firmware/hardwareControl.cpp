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
class HardwareControl
{
  HardwareControl::HardwareControl()
  {
    // static consts
    MOTOR_FEEDBACK_PIN = 3;
    RECIEVER_DRIVE_PIN = 4;
    MOTOR_SIGNAL_PIN = 5;
    SERVO_SIGNAL_PIN = 6;
    RECIEVER_STEER_PIN = 13;
    SERVO_FEEDBACK_PIN = 14;
    BUFFER_SIZE = 10;
    REPORT_DELAY = 10;

    // Reading motor feedback
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

    // Set up pins
    pinMode(MOTOR_FEEDBACK_PIN, INPUT;
    pinMode(RECIEVER_DRIVE_PIN, INPUT);
    pinMode(MOTOR_SIGNAL_PIN, OUTPUT);
    pinMode(SERVO_SIGNAL_PIN, OUTPUT);
    pinMode(RECIEVER_STEER_PIN, INPUT);
    pinMode(SERVO_FEEDBACK_PIN, INPUT);

    // Set up the interrupts
    cli();
    PCICR |= (1 << 0);  // turn on port B
    PCICR |= (1 << 2);  // Turn on port D
    PCMSK0 |= (1 << 6);   // Turn on PB5
    PCMSK2 |= (1 << 5);   // Turn on PD4

    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function below
    OCR0A = 0xAF;
    TIMSK0 |= (1 << OCIE0A);

    attachInterrupt(1, this.updateMotorSpeed, FALLING);
    sei();
  }

  HardwareControl::~HardwareControl() = default();

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
}

// Port B callback, PB5, pin 13, reciever steer
ISR(PCINT0_vect)
{
  // Get the time. Resolution = 8 us
  unsigned long time = micros();
  if(digitalRead(RECIEVER_STEER_PIN) == LOW)
  {
    // This is the falling edge, calculate effort
    float effort = (time - last_steer_edge_us - 1500) / 500.0;
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
    float effort = (time - last_drive_edge_us - 1500) / 500.0;
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
