### Motor control
This application creates three micro-ROS nodes, one for eache of the actuator controls on the boat. Sail control, rudder control and furl control. Ful control is not yet implemented. All nodes work in the same way on how to adjust the angle of the actuating part, They take in an angle and set the motor/ actuator to the wanted angle.


## Sail Control
Sail control takes an angle between -90 and 90 degrees. The controller taks in considereration how far of a distance the boom arm has to move and will accelerate accordingly. The Stepper motor which controls the mechanish has hogher torque on lower RPM, standard characteristics of a stepper motor.

## Rudder Control
Rudder control takes an angle between -45 and 45 degrees, if an angle large or lower than the interval it will be set to the lower or higher angle resepctively. The motor is a complete solution from Stegia, it works by supplying the motor with a control signal between 0-10V. the voltage is maped to an angle interval set in the motor. In our case the angle interval is 0-270 degrees, the reason behind the larg interval is because the esp32 is only capable of outputting 3,3v by itself. This results in a max angle of 3,3/10 * 270 = 90, which satisfies our requirements of +-45 degrees of rudder control. The function itself just takes an angle and convertes it to a voltage between 0-3,3V which is outputted by the internal dac on pin 25.

## True sail angle


The application subscribes to /position/SAIL_ANGLE and /rudder/ANGLE.

WIP