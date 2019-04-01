// Based on https://github.com/jetsonhacks/JHLEDBackpack/tree/master/src
// See https://elinux.org/Interfacing_with_I2C_Devices too
// As well as https://github.com/k-sheridan/HighlyAutonomousAerialReconnaissanceRobot/blob/master/Alpha/Current%20Code/Flight%20Control%20Software/Quadcopter_Main_rev13/IMU_Read.ino

#include <I2CInterface.h>

I2CInterface::I2CInterface(int address) {
  i2cBus = 1;
  i2cAddress = address;
}

I2CInterface::~I2CInterface() {
  closeDevice();
}

bool I2CInterface::openDevice() {
  char fileNameBuffer[32];
  sprintf(filenameBuffer, "/dev/i2c-%d", i2cBus);
  fileHandle = open(filenameBuffer, O_RDWR);
  if (fileHandle < 0) {
    // could not open the bus
    printf("Failed to open the bus.");
    return false;
   }

  if (ioctl(fileHandle, I2C_SLAVE, i2cAddress) < 0) {
    // could not open the device on the bus
    printf("Failed to acquire bus access and/or talk to slave.\n");
    return false;
  }
  return true;
}

void I2CInterface::closeDevice() {
  if(fileHandle > 0)  {
    close(fileHandle);
    fileHandle = -1;
  }
}

//int MPU6050::i2cwrite(int writeRegister, int writeValue) {
//  int retval = i2c_smbus_write_word_data(fileHandle, writeRegister, writeValue);
//  if (retval < 0) {
//      printf("MPU6050 Write error: %d", errno) ;
//      retval = -1 ;
//  }
//  return retval;
//}

int I2CInterface::i2cwrite(int writeValue) {
  int retval = i2c_smbus_write_byte(fileHandle, writeValue);
  if (retval < 0) {
      printf("MPU6050 Write error: %d", errno) ;
      retval = -1 ;
  }
  return retval;
}


int I2CInterface::i2cRead() {
  int retval = i2c_smbus_read_byte(fileHandle);
  if (retval < 0) {
    printf("MPU6050 Read error: %d", errno);
    retval = -1;
  }
  return retval;
}

//int MPU6050::i2cread(){
// char buffer[10] = {0};
//  float data;
//  char channel;
//  for (int i = 0; i<4; i++)
//    // Using I2C Read
//    if (read(file, buffer, 2) != 2) {
//      /* ERROR HANDLING: i2c transaction failed */
//      printf("Failed to read from the i2c bus.\n");
//      buffer = g_strerror(errno);
//      printf(buffer);
//      printf("\n\n");
//    } else {
//      data = (float)((buf[0] & 0b00001111)<<8)+buf[1];
//      data = data/4096*5;
//      channel = ((buf[0] & 0b00110000)>>4);
//      printf("Channel %02d Data:  %04f\n",channel,data);
//    }
//  }
//}
