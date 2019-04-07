#include <MPU6050.h>

MPU6050::MPU6050(int address) :
  imuHandle(address)
{
  initAccel();
  initGyro();
}

MPU6050::~MPU6050() = default;

void MPU6050::initGyro() {
  imuHandle.writeRegister(GYRO_CONFIG_REGISTER, GYRO_CONFIG_RANGE_2000);
  gyroRange = 2000;
}

void MPU6050::initAccel() {
  imuHandle.writeRegister(ACCEL_CONFIG_REGISTER, ACCEL_CONFIG_RANGE_16);
  accelRange = 16;
}

MPU6050::imu_data MPU6050::readIMU() {
  uint8_t burstData [IMU_BURST_BYTES] = {};
  imuHandle.readBurst(IMU_BURST_REGISTER, IMU_BURST_BYTES, burstData);

  MPU6050::imu_data measurement;

  measurement.accelerometer_x_mps = convertRawToEffort(burstData, ACCEL_REGISTER_X2 - IMU_BURST_REGISTER) * accelRange * ACCEL_CONSTANT_MPS;
  measurement.accelerometer_y_mps = convertRawToEffort(burstData, ACCEL_REGISTER_Y2 - IMU_BURST_REGISTER) * accelRange * ACCEL_CONSTANT_MPS;
  measurement.accelerometer_z_mps = convertRawToEffort(burstData, ACCEL_REGISTER_Z2 - IMU_BURST_REGISTER) * accelRange * ACCEL_CONSTANT_MPS;

  measurement.temperature_deg_c = convertRawToEffort(burstData, TEMP_REGISTER_2 - IMU_BURST_REGISTER) / TEMP_CONSTANT_DIV + TEMP_CONSTANT_ADD;

  measurement.gyro_x_radps = convertRawToEffort(burstData, GYRO_REGISTER_X2 - IMU_BURST_REGISTER) * gyroRange * GYRO_CONSTANT_RADPS;
  measurement.gyro_y_radps = convertRawToEffort(burstData, GYRO_REGISTER_Y2 - IMU_BURST_REGISTER) * gyroRange * GYRO_CONSTANT_RADPS;
  measurement.gyro_z_radps = convertRawToEffort(burstData, GYRO_REGISTER_Z2 - IMU_BURST_REGISTER) * gyroRange * GYRO_CONSTANT_RADPS;

  return measurement;
}

double MPU6050::convertRawToEffort(uint8_t * rawData, uint8_t startIndex)
{
  int16_t raw16 = ((int16_t)rawData[startIndex] << 8) | rawData[++startIndex];
  return ((double)raw16) / INT16_MAX;
}
