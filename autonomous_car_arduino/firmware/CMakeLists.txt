cmake_minimum_required(VERSION 2.8.3)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ROS_LIB_DIR}
)

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
# add_definitions(-DUSB_CON)

generate_arduino_firmware(hardware_control
  SRCS hardwareControl.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD mini328
  PORT /dev/ttyACM0   # Update later
)
