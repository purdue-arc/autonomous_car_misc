//  https://www.kernel.org/doc/Documentation/i2c/dev-interface
//  https://www.kernel.org/doc/Documentation/i2c/smbus-protocol

#include <imu_i2c/I2CInterface.h>
#include <stdio.h>

I2CInterface::I2CInterface(int bus, int address)
{
  this->m_bus = bus;
  this->m_address = address;
}

I2CInterface::~I2CInterface()
{
  closeDevice();
}

bool I2CInterface::openDevice()
{
  char filenameBuffer[32];
  sprintf(filenameBuffer, "/dev/i2c-%d", m_bus);
  m_handle = open(filenameBuffer, O_RDWR);
  if (m_handle < 0)
  {
    // could not open the bus
    perror("Failed to open the bus");
    return false;
   }
  if (ioctl(m_handle, I2C_SLAVE, m_address) < 0)
  {
    // could not open the device on the bus
    perror("Failed to acquire bus access and/or talk to slave");
    return false;
  }
  return true;
}

void I2CInterface::closeDevice()
{
  if(m_handle > 0)
  {
    close(m_handle);
    m_handle = -1;
  }
}

bool I2CInterface::writeRegister(uint8_t reg, uint8_t val)
{
  int retval = i2c_smbus_write_byte_data(m_handle, reg, val);
  if (retval < 0)
  {
      perror("MPU6050 Write error");
  }
  return retval == 0;
}

int I2CInterface::readRegister(uint8_t reg)
{
  int retval = i2c_smbus_read_byte_data(m_handle, reg);
  if (retval < 0)
  {
    perror("MPU6050 Read error");
  }
  return retval;
}

bool I2CInterface::readBurst(uint8_t reg, uint8_t num, uint8_t * vals)
{
  int retval = i2c_smbus_read_i2c_block_data(m_handle, reg, num, vals);
  if (retval < 0)
  {
    perror("MPU6050 Block read error");
  }
  return retval == num;
}
