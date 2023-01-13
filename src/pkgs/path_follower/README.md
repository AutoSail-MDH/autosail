# Path follower package
In this package there are two nodes contained in separate files. path_follower.py demo_path_traversal.py

## path_follower.py
The pathfollower.py script travels between two points and outputs a rudder angle as a ros2 /actuator/rudder messages.
path_follower.py takes two points it should travel between in longitude,latitude as a 
it also reads sensor inputs and outputs the desired angle to set the rudder

It takes inputs through subscription of ros2 message topics which include:
    The boats current pose: /position/pose
    The estimated true wind angle: /sensor/true_wind
    The measured wind angle: /sensor/wind
    The two points that are to be traveled between: /path/next_waypoint and /path/prev_waypoint

It publishes the desired rudder angle:
    Desired rudder angle: /actuator/rudder

## demo_path_traversal.py
The demo_path_traversal.py is only a demo code used for testing. It can be used to test the path_follower.py code, simulating sensor data and a path. It then automatically moves along the path changing its simulated position. 
The code starts with a list of coordinate waypoints and sets the current position of the boat at the first waypoint. The code then automatically traverses the list and sets it current position to the next waypoint. It also changes its heading to be directly towards the the next waypoint with a slight delay. 

It publishes simulated data on the following topics:
    /position/pose
    /sensor/true_wind
    /sensor/wind
    /path/next_waypoint and /path/prev_waypoint

## Setup

```bash
    cd ~/ros2_ws/autosail

    colcon build

    source install/local_setup.bash
```

## DEMO
To run the demo, two separate bash windows are needed for the two nodes. Make sure to do the setup first.

In the first window:
```bash
    cd ~/ros2_ws/autosail

    source install/local_setup.bash
    
    ros2 run pathfollower path_follower
```

In the second window:
```bash
    cd ~/ros2_ws/autosail

    source install/local_setup.bash
    
    ros2 run path_follower demo_path_traversal
```