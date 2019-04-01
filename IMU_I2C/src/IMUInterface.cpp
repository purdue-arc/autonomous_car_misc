#include <IMUInterface.h>

IMUInterface::IMUInterface() {
  gyroHandle = I2CInterface(ITG3205);
  accelHandle = I2CInterface(ADLX345);
}

IMUInterface::~IMUInterface = default();

void IMUInterface::initGyro() {
  gyroHandle.writeDevice(0x3E);
  gyroHandle.writeDevice(0x00);
  gyroHandle.writeDevice(0x15);
  gyroHandle.writeDevice(0x07);
  gyroHandle.writeDevice(0x17);
  gyroHandle.writeDevice(0x00);
}

void IMUInterface::initAccel() {
}

void IMUInterface::calibrateGyro(){
}

double[3] IMUInterface::readGyro() {
  gyroHandle.writeDevice(0x18);
  gyrohandle.readDevice()
}

