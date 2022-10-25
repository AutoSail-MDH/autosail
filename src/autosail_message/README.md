# Autosail ROS messages
This package contains the custom messages created to be used in the apps and ROS packages.

## Adding new messages
To add new messages, do the following:

Create the message file as seen in the /msg folder, then add the following line to the CMakeLists.txt file.
```
...
"msg/[MessageName].msg"
...
```
If the message is to be used in an app; copy the autosail_message folder and add it to the /firmware/mcu_ws folder. If already present, replace the old one. 