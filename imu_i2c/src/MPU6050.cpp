#include <MPU6050.h>

MPU6050::MPU6050() {
  gyroHandle = I2CInterface(ITG3205);
  accelHandle = I2CInterface(ADLX345);
}

MPU6050::~MPU6050 = default();

void MPU6050::initGyro() {
  imuHandle.writeRegister(GYRO_CONFIG, GYRO_CONFIG_RANGE_2000);



  gyroHandle.writeDevice(0x3E);
  gyroHandle.writeDevice(0x00);
  gyroHandle.writeDevice(0x15);
  gyroHandle.writeDevice(0x07);
  gyroHandle.writeDevice(0x17);
  gyroHandle.writeDevice(0x00);
}

void MPU6050::initAccel() {
}

void MPU6050::calibrateGyro() {
}

double[3] MPU6050::readGyro() {

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

  gyroHandle.writeDevice(0x18);
  gyrohandle.readDevice()
}
