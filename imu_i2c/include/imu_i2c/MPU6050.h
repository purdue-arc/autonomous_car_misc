#include <MPU6050.h>

class MPU6050
{
public:
  MPU6050();
  ~MPU6050();

  void initGyro();
  double[3] readGyro();
  void calibrateGyro();

  void initAccel();
  double[3] readAccel();
private:
  I2CInterface gyroHandle;
  I2CInterface accelHandle;

  // Gyro init
  static const uint8_t GYRO_CONFIG = 0x1B;
  static const uint8_t GYRO_CONFIG_RANGE_250 = 0b00000000;
  static const uint8_t GYRO_CONFIG_RANGE_500 = 0b00001000;
  static const uint8_t GYRO_CONFIG_RANGE_1000 = 0b00010000;
  static const uint8_t GYRO_CONFIG_2000 = 0b00011000;

  // Accel init
  static const uint8_t ACCEL_CONFIG = 0x1C;
  static const uint8_t ACCEL_CONFIG_RANGE_2 = 0b00000000;
  static const uint8_t ACCEL_CONFIG_RANGE_4 = 0b00001000;
  static const uint8_t ACCEL_CONFIG_RANGE_8 = 0b00010000;
  static const uint8_t ACCEL_CONFIG_16 = 0b00011000;

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
