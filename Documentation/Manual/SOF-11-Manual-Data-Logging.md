# Data-logging using ROS bag

Note: If topics cannot be discovered, make sure to follow [SOF.10-Manual-ROS2-BI-DIR-COM](https://github.com/AutoSail-MDH/AutoSail-HT21/blob/main/Documentation/Manual/SOF.10-Manual-ROS2-BI-DIR-COM.md).
Disclaimer: This manual assumes that topics have been initiated and are publishing data, although by default ROS bag utilises autodiscovery to start recording topics once initiated. Autodiscovery can be disabled by adding "--no-discovery" at EOL in launch file mentioned below.

The mentioned files can be found at [Autosail-HT21/ROS2/data_logging](https://github.com/AutoSail-MDH/AutoSail-HT21/tree/main/ROS2/data_logging)

## Record data

Run the following command to initiate recording of selected topics published on network.

1. Open "select_topics.txt" and add topic names, separated by rows

2. Launch recording by:

> ros2 launch record.launch.py

## Play data

Run the following command to initiate playback of selected topics recorded on network.

> ros2 launch play recorded_{timestamp}

## Echo topics

Once playback has been initiated, open new terminal and echo topics.

> ros2 topic echo /topic_1 /topic_2 /topic_3 ... /topic_N
