# autonomous_car_misc
This hosts the main Wiki for the car project, and it also has code specific to this platform

This specific code currently consists of:
 * autonomous_car_arduino: Low level hardware interface code running on the Arduino Pro Mini
 * autonomous_car_arduio_msgs: Required messages for autonomous_car_arduino
 * IMU_I2C: I2C driver, MPU6050 Library, and ROS wrapper to allow getting IMU data from MPU6050 over I2C
 * camera1394_launch: Launch files and parameters for running camera1394 on the Autonomous Car
