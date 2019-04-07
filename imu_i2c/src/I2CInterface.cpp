//  https://www.kernel.org/doc/Documentation/i2c/dev-interface
//  https://www.kernel.org/doc/Documentation/i2c/smbus-protocol

#include <I2CInterface.h>

I2CInterface::I2CInterface(int address)
{
  this.bus = 1;
  this.address = address;
}

I2CInterface::~I2CInterface()
{
  closeDevice();
}

bool I2CInterface::openDevice()
{
  char fileNameBuffer[32];
  sprintf(filenameBuffer, "/dev/i2c-%d", i2cBus);
  this.handle = open(filenameBuffer, O_RDWR);
  if (this.handle < 0)
  {
    // could not open the bus
    printf("Failed to open the bus.\n");
    return false;
   }
  if (ioctl(fileHandle, I2C_SLAVE, i2cAddress) < 0)
  {
    // could not open the device on the bus
    printf("Failed to acquire bus access and/or talk to slave.\n");
    return false;
  }
  return true;
}

void I2CInterface::closeDevice()
{
  if(this.handle > 0)
  {
    close(this.handle);
    this.handle = -1;
  }
}

bool I2CInterface::writeRegister(uint8_t reg, uint8_t val)
{
  int retval = i2c_smbus_write_byte_data(this.handle, reg, val);
  if (retval < 0)
  {
      printf("MPU6050 Write error: %d", errno);
  }
  return retval == 0;
}

int I2CInterface::readRegister(uint8_t reg)
{
  int retval = i2c_smbus_read_byte_data(this.handle, reg);
  if (retval < 0)
  {
    printf("MPU6050 Read error: %d", errno);
  }
  return retval;
}

bool I2CInterface::readBurst(uint8_t reg, uint8_t num, uint8_t * vals)
{
  int retval = i2c_smbus_read_i2c_block_data(this.handle, reg, num, vals);
  if (retval < 0)
  {
    printf("MPU6050 Read error: %d", errno);
  }
  return retval == num;
}
