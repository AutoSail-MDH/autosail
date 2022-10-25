# Data acquisition
This application bundles together three submodules, [GNSS Reading](https://github.com/AutoSail-MDH/autosail/src/apps/gnss_reading), [IMU Reading](https://github.com/AutoSail-MDH/autosail/src/apps/imu_reading) and [Wind Reading](https://github.com/AutoSail-MDH/autosail/src/apps/wind_reading). This is so all three can be ran on one MCU, instead of on seperate ones. See each seperate submodule for information on how to run them seperate.

This application is not used on the 4m boat.

## Hardware connection
The IMU and GPS are connected in series using the Qwiic connectors of the GPS, and the Wind sensor is connected directly to the ESP32.

### GPS -> ESP32

SDA(Blue cable) - GPIO 21 

SCL(Yellow cable) - GPIO 22 

3V3(Red cable) - 3V3

GND(Black cable) - GND

### IMU -> GPS

SDA - Blue cable

SCL - Yellow cable

Vin - Red cable

GND - Black cable

### Wind -> ESP32

RX(Yellow cable) - GPIO 4

TX(Green cable) - GPIO 5

VCC(Brown cable) - 5V

GND(White Cable) - GND
