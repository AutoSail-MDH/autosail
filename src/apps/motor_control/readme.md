# Motor control
This application creates micro-ROS nodes that takes an angle in radians and converts it into a PWM signal to be sent to the connected motors. The input values can be between -pi/2 and pi/2, but the physical servo does not quite achieve a full 180 degree rotation; but it is close enough that it should hardly make a difference. The outputted PWM singal is per the servo's specifications between 800 and 2200 microseconds. The version of ROS2 is Foxy on Ubuntu 20.04.

The sail PWM signal is sent to GPIO19 and the rudder PWM signal is sent to GPIO18, being pin 19 and 18 on the ESP32 respectively.

The application subscribes to /position/SAIL_ANGLE and /rudder/ANGLE.
