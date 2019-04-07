//  https://www.kernel.org/doc/Documentation/i2c/dev-interface
//  https://www.kernel.org/doc/Documentation/i2c/smbus-protocol

#include <I2CInterface.h>

I2CInterface::I2CInterface(int addr)
{
  bus = 1;
  address = addr;
}

I2CInterface::~I2CInterface()
{
  closeDevice();
}

bool I2CInterface::openDevice()
{
  char filenameBuffer[32];
  sprintf(filenameBuffer, "/dev/i2c-%d", bus);
  handle = open(filenameBuffer, O_RDWR);
  if (handle < 0)
  {
    // could not open the bus
    printf("Failed to open the bus.\n");
    return false;
   }
  if (ioctl(handle, I2C_SLAVE, address) < 0)
  {
    // could not open the device on the bus
    printf("Failed to acquire bus access and/or talk to slave.\n");
    return false;
  }
  return true;
}

void I2CInterface::closeDevice()
{
  if(handle > 0)
  {
    close(handle);
    handle = -1;
  }
}

bool I2CInterface::writeRegister(uint8_t reg, uint8_t val)
{
  int retval = i2c_smbus_write_byte_data(handle, reg, val);
  if (retval < 0)
  {
      printf("MPU6050 Write error: %d", errno);
  }
  return retval == 0;
}

int I2CInterface::readRegister(uint8_t reg)
{
  int retval = i2c_smbus_read_byte_data(handle, reg);
  if (retval < 0)
  {
    printf("MPU6050 Read error: %d", errno);
  }
  return retval;
}

bool I2CInterface::readBurst(uint8_t reg, uint8_t num, uint8_t * vals)
{
  int retval = i2c_smbus_read_i2c_block_data(handle, reg, num, vals);
  if (retval < 0)
  {
    printf("MPU6050 Read error: %d", errno);
  }
  return retval == num;
}
