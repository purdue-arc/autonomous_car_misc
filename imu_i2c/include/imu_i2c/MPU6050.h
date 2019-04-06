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
  static const uint8_t ACCEL_CONFIG_RANGE_250 = 0b00000000;
  static const uint8_t ACCEL_CONFIG_RANGE_500 = 0b00001000;
  static const uint8_t ACCEL_CONFIG_RANGE_1000 = 0b00010000;
  static const uint8_t ACCEL_CONFIG_2000 = 0b00011000;

  //gyro
  // 1B: bits 4 and 3
  // 0, 1, 2, 3:
  // 250, 500, 1000, 2000


  // Accel
  // read: 3B,3C,3D,3E,3F,40

  // Temp
  // 41,42
  // = value / 340 + 36.53

  // Gyro
  // read: 43,44,45,46,47,48

  // rate is reg 25

  // register 27 has scale

  // register 28 has scale
}
