# Data acquisition
This application bundles together three submodules, [GPS](https://github.com/AutoSail-MDH/AutoSail/tree/main/uros/src/apps/gps), [Vessel orientation](https://github.com/AutoSail-MDH/AutoSail/tree/main/uros/src/apps/vessel_orientation) and [Wind direction](https://github.com/AutoSail-MDH/AutoSail/tree/main/uros/src/apps/wind_direction). This is so all three can be ran on one MCU, instead of on seperate ones. See each seperate submodule for information on how to run them seperate.

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
