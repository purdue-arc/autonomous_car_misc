#include <imu_i2c/MPU6050.h>

MPU6050::MPU6050(int bus, int address) :
  imuHandle(bus, address)
{
  if (imuHandle.openDevice())
  {
    resetChip();
    wakeChip();
    initAccel();
    initGyro();
    printf("device opened and configured\n");
  }
  else
  {
    printf("device open failure\n");
  }
}

MPU6050::~MPU6050() = default;

void MPU6050::resetChip()
{
 imuHandle.writeRegister(CHIP_CONFIG_REGISTER, CHIP_CONFIG_RESET);
}

void MPU6050::wakeChip()
{
 imuHandle.writeRegister(CHIP_CONFIG_REGISTER, CHIP_CONFIG_AWAKE);
}

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

  measurement.accelerometer_x_mps = decodeBurstData(burstData, ACCEL_REGISTER_X2 - IMU_BURST_REGISTER) * accelRange * ACCEL_CONSTANT_MPS;
  measurement.accelerometer_y_mps = decodeBurstData(burstData, ACCEL_REGISTER_Y2 - IMU_BURST_REGISTER) * accelRange * ACCEL_CONSTANT_MPS;
  measurement.accelerometer_z_mps = decodeBurstData(burstData, ACCEL_REGISTER_Z2 - IMU_BURST_REGISTER) * accelRange * ACCEL_CONSTANT_MPS;

  measurement.temperature_deg_c = decodeBurstData(burstData, TEMP_REGISTER_2 - IMU_BURST_REGISTER) / TEMP_CONSTANT_DIV + TEMP_CONSTANT_ADD;

  measurement.gyro_x_radps = decodeBurstData(burstData, GYRO_REGISTER_X2 - IMU_BURST_REGISTER) * gyroRange * GYRO_CONSTANT_RADPS;
  measurement.gyro_y_radps = decodeBurstData(burstData, GYRO_REGISTER_Y2 - IMU_BURST_REGISTER) * gyroRange * GYRO_CONSTANT_RADPS;
  measurement.gyro_z_radps = decodeBurstData(burstData, GYRO_REGISTER_Z2 - IMU_BURST_REGISTER) * gyroRange * GYRO_CONSTANT_RADPS;

  return measurement;
}

double MPU6050::decodeBurstData(uint8_t * rawData, uint8_t startIndex)
{
  const int16_t raw16 = (static_cast<int16_t>(rawData[startIndex]) << 8) | rawData[++startIndex];
  return static_cast<double>(raw16) / INT16_MAX;
}
