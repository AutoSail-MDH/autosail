# Data-logging using ROS bag

Note: If topics cannot be discovered, make sure to follow [SOF.10-Manual-ROS2-BI-DIR-COM](https://github.com/AutoSail-MDH/AutoSail-HT21/Documentation/Manual/SOF.10-Manual-ROS2-BI-DIR-COM.md).
Disclaimer: This manual assumes that topics have been initiated and are publishing data, although by default ROS bag utilises autodiscovery to start recording topics once initiated. Autodiscovery can be disabled by adding "--no-discovery" at EOL in launch file mentioned below.

The mentioned XML files can be found at [Autosail-HT21/ROS2/data_logging](https://github.com/AutoSail-MDH/AutoSail-HT21/ROS2/data_logging) but do not need to be standalone launch files for logging, meaning that the command contained within mentioned files can be added to a larger project launch file.

## Record data

Run the following command to initiate recording of all topics published on network.

> ros2 launch record_all.launch.xml

```xml
<launch>
  <executable cmd="ros2 bag record -a -o all_data" output="screen" />
</launch>
```

1. Add critical topics to record_critical.launch.xml
2. Run the following command to initiate recording of specific topics (critical) published on network.

> ros2 launch record_critical.launch.xml

```xml
<launch>
  <executable cmd="ros2 bag record -o critical_data /topic_1 /topic_2 /topic_3" output="screen" />
</launch>
```

## Play data

Run the following command to initiate playback of all topics recorded on network.

> ros2 launch play_all.launch.xml

```xml
<launch>
  <executable cmd="ros2 bag play all_data" output="screen" />
</launch>
```

Run the following command to initiate playback of critical topics recorded on network.

> ros2 launch play_all.launch.xml

```xml
<launch>
  <executable cmd="ros2 bag play critical_data" output="screen" />
</launch>
```

## Echo topics

Once playback launch file has been initiated, open new terminal and echo topics.

> ros2 topic echo /topic_1 /topic_2 /topic_3 ... /topic_N

OR see [SOF-12-Data-Visualisation](https://github.com/AutoSail-MDH/AutoSail-HT21/Documentation/Manual/SOF-12-Data-Visualisation.md) to visualise topics using GUI.
