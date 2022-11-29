# Data acquisition
This application is used to run the angle sensor.

## Hardware connection
The angle sensor form Pepperl+Fuchs is connected to a power source. The input voltage shuld be 18-30V but everything above 8 also works:
Brown cable - Positive charge
Blue cable - Negative charge

The INA219 current sensor is connected directly to the ESP32 via a qwiic connector(I2C input!):
SDA(Blue cable) - GPIO 21 
SCL(Yellow cable) - GPIO 22 
3-5DC(Red cable) - 3V3
GND(Black cable) - GND

The INA219 is also connected to the angle sensor:
Vin+ - Grey cable
Vin- - Blue cable