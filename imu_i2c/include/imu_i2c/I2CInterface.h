#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <stdint.h>

class I2CInterface
{
public:
  I2CInterface(int address);
  ~I2CInterface();

  bool openDevice();
  void closeDevice();

  int readRegister();
  int writeRegister();
  int readBurst();

private:
  int handle;
  int address;
  int bus;
}
