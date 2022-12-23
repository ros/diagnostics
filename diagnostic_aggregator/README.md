# Overview
This package contains the `aggregator_node`.
It listens to the [`diagnostic_msgs/DiagnosticArray`](https://index.ros.org/p/diagnostic_msgs) messages on the `/diagnostics` topic and aggregates and published them on the `/diagnostics_agg` topic.

## Example
One example of how to use this package is to aggregate the diagnostics of a robot.
Aggregation means that the diagnostics of the robot are grouped by various aspects, like their location on the robot, their type, etc.
This will allow you to easily see which part of the robot is not working properly.

In our example, we are looking at a robot with arms and legs.
The robot has two of each, one on each side.
The robot also 4 camera sensors, one left and one right and one in the front and one in the back.
These are all the available diagnostic sources:

```
/arms/left/motor
/arms/right/motor
/legs/left/motor
/legs/right/motor
/sensors/left/cam
/sensors/right/cam
/sensors/front/cam
/sensors/rear/cam
```

We want to group the diagnostics by
- all sensors
- all motors
- left side of the robot
- right side of the robot

We can achieve that by creating a configuration file that looks like this (see [example_analyzers.yaml](diagnostic_aggregator/example/example_analyzers.yaml)):
```
analyzers:
  ros__parameters:
    path: Aggregation
    arms:
      type: diagnostic_aggregator/GenericAnalyzer
      path: Arms
      startswith: [ '/arms' ]
    legs:
      type: diagnostic_aggregator/GenericAnalyzer
      path: Legs
      startswith: [ '/legs' ]
    sensors:
      type: diagnostic_aggregator/GenericAnalyzer
      path: Sensors
      startswith: [ '/sensors' ]
    motors:
      type: diagnostic_aggregator/GenericAnalyzer
      path: Motors
      contains: [ '/motor' ]
    topology:
      type: 'diagnostic_aggregator/AnalyzerGroup'
      path: Topology
      analyzers:
        left:
          type: diagnostic_aggregator/GenericAnalyzer
          path: Left
          contains: [ '/left' ]
        right:
          type: diagnostic_aggregator/GenericAnalyzer
          path: Right
          contains: [ '/right' ]
```

This will allow [rqt_robot_monitor](https://index.ros.org/p/rqt_robot_monitor) to display the diagnostics in a more useful way:
![doc/rqt_robot_monitor.png](doc/rqt_robot_monitor.png)

Note that it will also display the highest state per group to allow you to see at a glance which part of the robot is not working properly.
For example in the above image, the left side of the robot is not working properly, because the left cam is in the `ERROR` state.

# Analyzers
