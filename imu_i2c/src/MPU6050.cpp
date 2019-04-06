#include <MPU6050.h>

MPU6050::MPU6050() {
  gyroHandle = I2CInterface();
  initAccel();
  initGyro();
}

MPU6050::~MPU6050 = default();

void MPU6050::initGyro() {
  imuHandle.writeRegister(GYRO_CONFIG_REGISTER, GYRO_CONFIG_2000);
  gyroRange = 2000;
}

void MPU6050::initAccel() {
  imuHandle.writeRegister(ACCEL_CONFIG_REGISTER, ACCEL_CONFIG_RANGE_16);
  accelRange = 16;
}

imu_data MPU6050::readIMU() {
  uint8_t burstData [IMU_BURST_BYTES] = {};
  imuHandle.readBurst(IMU_BURST_REGISTER, IMU_BURST_BYTES, burstData);

  imu_data measurement;

  measurement.accelerometer_x_mps = convertRawToEffort(burstData, ACCEL_REGISTER_X2 - IMU_BURST_BYTES) * accelRange * ACCEL_CONSTANT_MPS;
  measurement.accelerometer_y_mps = convertRawToEffort(burstData, ACCEL_REGISTER_Y2 - IMU_BURST_BYTES) * accelRange * ACCEL_CONSTANT_MPS;
  measurement.accelerometer_z_mps = convertRawToEffort(burstData, ACCEL_REGISTER_Z2 - IMU_BURST_BYTES) * accelRange * ACCEL_CONSTANT_MPS;

  measurement.temperature_deg_c = convertRawToEffort(burstData, TEMP_REGISTER_2 - IMU_BURST_BYTES) / TEMP_CONSTANT_DIV + TEMP_CONSTANT_ADD;

  measurement.gyro_x_radps = convertRawToEffort(burstData, GYRO_REGISTER_X2 - IMU_BURST_BYTES) * gyroRange * GYRO_CONSTANT_RADPS;
  measurement.gyro_y_radps = convertRawToEffort(burstData, GYRO_REGISTER_Y2 - IMU_BURST_BYTES) * gyroRange * GYRO_CONSTANT_RADPS;
  measurement.gyro_z_radps = convertRawToEffort(burstData, GYRO_REGISTER_Z2 - IMU_BURST_BYTES) * gyroRange * GYRO_CONSTANT_RADPS;

  return measurement;
}

double convertRawToEffort(uint8_t (&rawData) [], uint8_t startIndex)
{
  int16_t raw16 = ((int16_t)rawData(startIndex) << 8) | rawData(++startIndex);
  if (raw16 < 0)
  {
    return ((double)raw16) / -INT16_MIN
  }
  else
  {
    return ((double)raw16) / INT16_MAX
  }
}
