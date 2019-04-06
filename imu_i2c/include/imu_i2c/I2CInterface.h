# include <linux/i2c-dev.h>
# include <i2c/smbus.h>
# include <errno.h>
# include <sys/ioctl.h>

class I2CInterface
{
public:
  I2CInterface(int address);
  ~I2CInterface();

  bool openDevice();
  void closeDevice();

  int writeDevice();
  int readDevice();

private:
  int fileHandle;
  int i2cAddress;
  int i2cBus;
}
