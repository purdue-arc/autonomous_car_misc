# autonomous_car_arduino
Arduino code running the hardware interface for the autonomous car

### Uploading
catkin build --no-deps  autonomous_car_arduino --make-args autonomous_car_arduino_firmware_hardware_control-upload

### Testing
rosrun rosserial_python serial_node.py /dev/ttyACM0
