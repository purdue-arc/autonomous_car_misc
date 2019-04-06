#include <Arduino.h>
#include <Servo.h>  // Uses timer 1
#include <avr/interrupt.h>

class HardwareControl
{
public:

  HardwareControl();
  ~HardwareControl();

  // Determine average value based off pointer to buffer
  float getBufferAverage(float * buffer);

  // ROS callbacks
  void controlCar(const autonomous_car_arduino_msgs::ArduinoCommand& msg_command);
  void populateReport(autonomous_car_arduino_msgs::ArduinoReport& msg_report);

  // Interrupts
  void updateMotorSpeed();
  // Port B callback, PB5, pin 13, reciever steer
  // Port D callback, PD4, pin 4, reciever drive

  // Pin assignments
  static const byte MOTOR_FEEDBACK_PIN;
  static const byte RECIEVER_DRIVE_PIN;
  static const byte MOTOR_SIGNAL_PIN;
  static const byte SERVO_SIGNAL_PIN;
  static const byte RECIEVER_STEER_PIN ;
  static const byte SERVO_FEEDBACK_PIN;
  static const byte BUFFER_SIZE;

private:
  volatile unsigned long last_motor_edge_us;
  volatile float motor_feedback_buffer [];
  volatile byte motor_buffer_index;

  // Reading input from RC controller
  volatile unsigned long last_steer_edge_us;
  volatile float steer_effort_buffer [];
  volatile byte steer_buffer_index;

  volatile unsigned long last_drive_edge_us;
  volatile float drive_effort_buffer [];
  volatile byte drive_buffer_index;
}
