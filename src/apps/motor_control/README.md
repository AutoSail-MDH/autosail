# Motor control
This application creates three micro-ROS nodes, one for each of the actuators on the boat. Sail control, rudder control and furl control. Furl control is not yet implemented. All nodes work in the same way on how to adjust the angle of the actuating part, They take in an angle and set the motor/ actuator to the wanted angle.


## Sail Control
Sail control takes an angle between -90 and 90 degrees. The controller taks in considereration how far of a distance the boom arm has to move and will accelerate and move accordingly. To know how far the boom arm has to move a angle sensor is used to measure the angle of the boom arm, this is the true sail angle. The true sail angle is explanied below. The Stepper motor which controls the mechanish has higher torque on lower RPM, standard characteristics of a stepper motor. The stepper motor is connected to a stepper motor driver and the driver is connectet to the ESP32 with four cables

### Hardware Connection
#### ESP32 -> Motor driver:
GPIO19 -> Step+
GPIO18 -> Dir+
GND -> Black/Yellow -> Step- and Dir-

#### ESP32 -> Angle sensor/Current sensor
GPIO21 -> SDA
GPIO22 -> SCL
GND -> GND


## Rudder Control
Rudder control takes an angle between -45 and 45 degrees, if an angle large or lower than the interval it will be set to the lower or higher angle resepctively. The motor is a complete solution from Stegia, it works by supplying the motor with a control signal between 0-10V. the voltage is maped to an angle interval set in the motor. In our case the angle interval is 0-270 degrees, the reason behind the larg interval is because the esp32 is only capable of outputting 3,3v by itself. This results in a max angle of 3,3/10 * 270 = 90, which satisfies our requirements of +-45 degrees of rudder control. The function itself just takes an angle and convertes it to a voltage between 0-3,3V which is outputted by the internal dac on pin 25.

### Hardware Connection
DAC 1 (PIN 25) -> Green cable
Common GND -> Black cable
DC/DC 12V -> Red cable



## True sail angle

### Hardware Connection






The application subscribes to /actuator/rudder, /actuator/sail and /actuator/furl

WIP