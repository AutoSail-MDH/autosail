# Data acquisition
[Vessel orientation](https://github.com/AutoSail-MDH/AutoSail/tree/main/uros/src/apps/vessel_orientation)  This is run on one microcontroller

## Hardware connection
The IMU sensor is connected directly to the ESP32.

### IMU -> ESP32

SDA(Blue cable) - GPIO 21 

SCL(Yellow cable) - GPIO 22 

3V3(Red cable) - 3V3

GND(Black cable) - GND

## Calibration
The IMU needs to be calibrated everytime it is restarted. This is done automatically by the BNO055 IMU sensor and should not take more than a couple of seconds. However if the IMU shows incorrect values for a longer time run the "imu_calibration" app. This app gives new offset values which are to be inserted into the "app.cpp" code. It is recommended to do this before every real test.

### The cailbration processes are:

For the gyroscope the IMU must be held still in any position. Test different positions if the first one does not work.

For the magnetometer the IMU needs to be moved in random directions. This one generally takes the longest time to be fully calibrated and can take several minutes. 

For the accelerometer the IMU needs to be in 6 standing positions: -x, +x, -y, +y, -z and +z.
