#include <linux/i2c-dev.h>
//#include <i2c/smbus.h>
//#include <i2c-tools/i2c-dev.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdio.h>

class I2CInterface
{
public:
  I2CInterface(int addr);
  ~I2CInterface();

  bool openDevice();
  void closeDevice();

  int readRegister(uint8_t reg);
  bool writeRegister(uint8_t reg, uint8_t val);
  bool readBurst(uint8_t reg, uint8_t num, uint8_t * vals);

private:
  int handle;
  int address;
  int bus;
};
