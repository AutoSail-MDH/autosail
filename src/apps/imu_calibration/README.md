# IMU Calibration
This app contains calibration code for the IMU sensor. The goal of this app is to generate offset data which is to be manually inserted into the "imu_reading" app during initialization.

## BNO055
The BNO055 IMU sensor has three sensors: gyroscope, accelerometer and magnetometer. Every sensor is continously calibrated during runtime by internal algorithms. Sensor offsets can be set to speed up this calibration.  

## Hardware connection
The IMU sensor is connected directly to the ESP32.

### IMU -> ESP32

SDA(Blue cable) - GPIO 21 

SCL(Yellow cable) - GPIO 22 

3V3(Red cable) - 3V3

GND(Black cable) - GND

## Calibration

Start the app.

In another terminal monitor the "/sensor/imu_calibration" topic

Use the following "calibration processes" to calibrate the IMU.

The progress for each sensor is shown as a number from 0-3 with 3 being fully calibrated:
"calibration_gyro" shows the progress for the gyro.
"calibration_magnetometer" shows the progress for the magnetometer.
"calibration_accelerometer" shows the progress for the accelerometer.

When the IMU is calibrated then "is_calibration_complete" is set to 1. 


### The cailbration processes are:

For the gyroscope the IMU must be layed completely still in any position. Test different positions if the first one does not work.

For the magnetometer the IMU needs to be moved in random directions. 

For the accelerometer the IMU needs to be in 6 standing positions: -x, +x, -y, +y, -z and +z.
