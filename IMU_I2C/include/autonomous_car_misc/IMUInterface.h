#include <I2CInterface.h>

class IMUInterface
{
public:
  IMUInterface();
  ~IMUInterface();

  void initGyro();
  double[3] readGyro();
  void calibrateGyro();

  void initAccel();
  double[3] readAccel();
private:
  I2CInterface gyroHandle;
  I2CInterface accelHandle;
}
