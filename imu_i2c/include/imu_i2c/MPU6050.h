#include <stdint.h>
#include "I2CInterface.h"

class MPU6050
{
public:

  struct imu_data
  {
    double accelerometer_x_mps;
    double accelerometer_y_mps;
    double accelerometer_z_mps;
    double temperature_deg_c;
    double gyro_x_radps;
    double gyro_y_radps;
    double gyro_z_radps;
  }

  MPU6050();
  ~MPU6050();

  void initGyro();
  void initAccel();

  imu_data MPU6050::readIMU()
private:

  double convertRawToEffort(uint8_t (&rawData) [], uint8_t startIndex);

  I2CInterface imuHandle;
  double gyroRange;
  double accelRange;

  // from gees to m/s
  static const double ACCEL_CONSTANT_MPS = 9.81;
  // from deg / s to rads / s
  static const double GYRO_CONSTANT_RADPS = 57.2957795;

  // from unknown to deg c
  static const double TEMP_CONSTANT_DIV = 340;
  static const double TEMP_CONSTANT_ADD = 36.53;

  // Gyro init
  static const uint8_t GYRO_CONFIG_REGISTER = 0x1B;
  static const uint8_t GYRO_CONFIG_RANGE_250 = 0b00000000;
  static const uint8_t GYRO_CONFIG_RANGE_500 = 0b00001000;
  static const uint8_t GYRO_CONFIG_RANGE_1000 = 0b00010000;
  static const uint8_t GYRO_CONFIG_RANGE_2000 = 0b00011000;

  // Accel init
  static const uint8_t ACCEL_CONFIG_REGISTER = 0x1C;
  static const uint8_t ACCEL_CONFIG_RANGE_2 = 0b00000000;
  static const uint8_t ACCEL_CONFIG_RANGE_4 = 0b00001000;
  static const uint8_t ACCEL_CONFIG_RANGE_8 = 0b00010000;
  static const uint8_t ACCEL_CONFIG_RANGE_16 = 0b00011000;

  // Burst read
  static const uint8_t IMU_BURST_REGISTER = 0x3B;
  static const uint8_t IMU_BURST_BYTES = 14;

  // Accel read
  static const uint8_t ACCEL_REGISTER_X2 = 0x3B;
  static const uint8_t ACCEL_REGISTER_X1 = 0x3C;
  static const uint8_t ACCEL_REGISTER_Y2 = 0x3D;
  static const uint8_t ACCEL_REGISTER_Y1 = 0x3E;
  static const uint8_t ACCEL_REGISTER_Z2 = 0x3F;
  static const uint8_t ACCEL_REGISTER_Z1 = 0x40;

  // Temp read
  static const uint8_t TEMP_REGISTER_2 = 0x41;
  static const uint8_t TEMP_REGISTER_1 = 0x42;

  // Gyro read
  static const uint8_t GYRO_REGISTER_X2 = 0x43;
  static const uint8_t GYRO_REGISTER_X1 = 0x44;
  static const uint8_t GYRO_REGISTER_Y2 = 0x45;
  static const uint8_t GYRO_REGISTER_Y1 = 0x46;
  static const uint8_t GYRO_REGISTER_Z2 = 0x47;
  static const uint8_t GYRO_REGISTER_Z1 = 0x48;
}
