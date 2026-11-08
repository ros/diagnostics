General information about this repository, including legal information and known issues/limitations, are given in [README.md](../README.md) in the repository root.

# The diagnostic_aggregator package

This package contains the `aggregator_node`.
It listens to the [`diagnostic_msgs/DiagnosticArray`](https://index.ros.org/p/diagnostic_msgs) messages on the `/diagnostics` topic and aggregates and published them on the `/diagnostics_agg` topic.

One use case for this package is to aggregate the diagnostics of a robot.
Aggregation means that the diagnostics of the robot are grouped by various aspects, like their location on the robot, their type, etc.
This will allow you to easily see which part of the robot is causing problems.

## Example

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

We can achieve that by creating a configuration file that looks like this (see [example_analyzers.yaml](example/example_analyzers.yaml)):

``` yaml
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

Based on this configuration, the [rqt_robot_monitor](https://index.ros.org/p/rqt_robot_monitor) will display the diagnostics information in a well-arranged manner as follows:
![doc/rqt_robot_monitor.png](doc/rqt_robot_monitor.png)

Note that it will also display the highest state per group to allow you to see at a glance which part of the robot is not working properly.
For example in the above image, the left side of the robot is not working properly, because the left camera is in the `ERROR` state.

# Analyzers

The `aggregator_node` will load analyzers to process the diagnostics data.
An analyzer is a plugin that inherits from the [`diagnostic_aggregator::Analyzer`](include/diagnostic_aggregator/analyzer.hpp) class.
Analyzers must be implemented in packages that directly depend on [`pluginlib`](https://index.ros.org/p/pluginlib) and `diagnostic_aggregator`.

The [`diagnostic_aggregator::Analyzer`](include/diagnostic_aggregator/analyzer.hpp) class is purely virtual and derived classes must implement the following methods:

- `init()` - Analyzer is initialized with base path and namespace
- `match()` - Returns true if the analyzer is interested in the status message
- `analyze()` - Returns true if the analyzer will analyze the status message
- `report()` - Returns results of analysis as vector of status messages
- `getPath()` - Returns the prefix path of the analyzer (e.g., "/robot/motors/")
- `getName()` - Returns the name of the analyzer (e.g., "Motors")

Analyzers can choose the value of the error level of their output.
Usually, the error level of the output is the highest error level of the input.
The analyzers are responsible for setting the name of each item in the output correctly.

# Using the aggregator_node

## Configuration

The `aggregator_node` can be configured at launch time like in this example:

``` yaml
pub_rate: 1.0 # Optional, defaults to 1.0
base_path: 'PRE' # Optional, defaults to ""
analyzers:
  motors:
    type: PR2MotorsAnalyzer
  joints:
    type: GenericAnalyzer
    path: 'Joints'
    regex: 'Joint*'
```

The `pub_rate` parameter is the rate at which the aggregated diagnostics will be published.
The `base_path` parameter is the prefix that will be added to the name of each item in the output.

Under the `analyzers` key, you can specify the analyzers that you want to use.
Each analyzer must have a unique name.
Under the name, you must specify the type of the analyzer.
This must be the name of the class that implements the analyzer.
Additional parameters depend on the type of the analyzer.

Any diagnostic item that is not matched by any analyzer will be published by an "Other" analyzer.
Items created by the "Other" analyzer will go stale after 5 seconds, unless they are marked persistent (see below).

The `critical` parameter makes the aggregator react immediately to a degradation in diagnostic state.
This is useful if the toplevel state is parsed by a watchdog for example.

## Persistent items

By default an analyzer decides staleness purely from the age of the last received message: an item that has not been updated within `timeout` is reported as `STALE` (or dropped when `discard_stale` is set), and the "Other" analyzer drops items after 5 seconds.
A publisher can opt out of this for a specific status by adding the reserved key-value pair `persistent_state: True` to `DiagnosticStatus.values` (`DiagnosticStatusWrapper::setPersistentState()` in `diagnostic_updater` does exactly that).

A persistent item

- keeps its last level and message in the aggregated output, so it is never turned `STALE` or discarded,
- gets an extra `last_update_age` value (seconds since the last update), so operators can still tell an old persistent state from a fresh one,
- goes back to the normal timeout behaviour as soon as the publisher sends it again without the key.

This is intended for terminal or latched states, for example the final "Node shutting down" status a node publishes before it exits, or an emergency stop that stays latched.
It is not meant for ordinary operation: a persistent item whose publisher died keeps showing its last level until the aggregator is restarted.
Persistence only covers the lifetime of the aggregator, since `/diagnostics` is not published with transient-local durability.

`GenericAnalyzer` accepts an optional `persistent_timeout` parameter (seconds, default unlimited) to cap how long a persistent item is honoured; after that it is treated like any other item.
Persistent items that end up in the "Other" analyzer are capped by the aggregator-level `other_persistent_timeout` parameter (seconds, default unlimited) instead, since the "Other" analyzer has no configuration of its own.

``` yaml
analyzers:
  ros__parameters:
    other_persistent_timeout: 3600.0 # Optional, cap for persistent items in "Other"
    motors:
      type: diagnostic_aggregator/GenericAnalyzer
      path: Motors
      contains: [ 'motor_driver: ' ]
      timeout: 5.0
      persistent_timeout: 3600.0 # Optional, keep a persistent item for at most one hour
```

## Launching

You can launch the `aggregator_node` like this (see [example.launch.py.in](example/example.launch.py.in)):

``` python
    aggregator = launch_ros.actions.Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        output='screen',
        parameters=[analyzer_params_filepath])
    return launch.LaunchDescription([
        aggregator,
    ])
```

You can add analyzers at runtime using the `add_analyzer` node like this (see [example.launch.py.in](example/example.launch.py.in)):

```
    add_analyzer = launch_ros.actions.Node(
        package='diagnostic_aggregator',
        executable='add_analyzer',
        output='screen',
        parameters=[add_analyzer_params_filepath])
    return launch.LaunchDescription([
        add_analyzer,
    ])
```

This node updates the parameters of the `aggregator_node` by calling the service `/analyzers/set_parameters_atomically`.
The `aggregator_node` will detect when a `parameter-event` has introduced new parameters to it.
When this happens the `aggregator_node` will reload all analyzers based on its new set of parameters.
Adding analyzers this way can be done at runtime and can be made conditional.

In the example, `add_analyzer` will add an analyzer for diagnostics that are marked optional:

``` yaml
/**:
  ros__parameters:
    optional:
      type: diagnostic_aggregator/GenericAnalyzer
      path: Optional
      startswith: [ '/optional' ]
```

This will move the `/optional/runtime/analyzer` diagnostic from the "Other" to  "Aggregation" where it will not go stale after 5 seconds and will be taken into account for the toplevel state.

# Basic analyzers

The `diagnostic_aggregator` package provides a few basic analyzers that you can use to aggregate your diagnostics.

## GenericAnalyzer

The [`diagnostic_aggregator::GenericAnalyzer`](include/diagnostic_aggregator/generic_analyzer.hpp) class is a basic analyzer that can be configured to match diagnostics based on their name.
By defining a `path` parameter, you can specify the prefix that will be added to the name of each item in the output.
This way you can group diagnostics by their location or other aspects as demonstrated in the [example](#example).

## AnalyzerGroup

The [`diagnostic_aggregator::AnalyzerGroup`](include/diagnostic_aggregator/analyzer_group.hpp) class is a basic analyzer that can be configured to group other analyzers.
It has itself an `analyzers` parameter that can be filled with other analyzers to group them.

An example of this is (see [example_analyzers.yaml](diagnostic_aggregator/example/example_analyzers.yaml)):

``` yaml
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

## DiscardAnalyzer

The [`diagnostic_aggregator::DiscardAnalyzer`](include/diagnostic_aggregator/discard_analyzer.hpp) class is a basic analyzer that discards all diagnostics that match it.
This can be useful if you want to ignore some diagnostics.

## IgnoreAnalyzer

The [`diagnostic_aggregator::IgnoreAnalyzer`](include/diagnostic_aggregator/ignore_analyzer.hpp) will ignore all parameters in its namespace and not match anything.

The difference between the `DiscardAnalyzer` and the `IgnoreAnalyzer` is that the `DiscardAnalyzer` will match diagnostics and discard them, while the `IgnoreAnalyzer` will not match anything.
This means that things that are ignored by the `IgnoreAnalyzer` will still be published in the "Other" analyzer, while things that are discarded by the `DiscardAnalyzer` will not be published at all.

# ROS API

## `aggregator_node`

### Subscribed Topics

- `diagnostics` ([diagnostic_msgs/DiagnosticArray](https://index.ros.org/p/diagnostic_msgs)) - The diagnostics to be aggregated

### Published Topics

- `diagnostics_agg` ([diagnostic_msgs/DiagnosticArray](https://index.ros.org/p/diagnostic_msgs)) - The aggregated diagnostics
- `diagnostics_toplevel_state` ([diagnostic_msgs/DiagnosticStatus](https://index.ros.org/p/diagnostic_msgs)) - The highest state of the aggregated diagnostics

### Parameters

- `*` (map, default: {}) - The analyzers that will be used to aggregate the diagnostics
- `pub_rate` (double, default: 1.0) - The rate at which the aggregated diagnostics will be published
- `path` (string, default: "") - The prefix that will be added to the name of each item in the output
- `other_as_errors` (bool, default: false) - If the 'Other' aggregator (that matches all statuses not matched by any declared aggregator) should treat everything as ERROR.
- `history_depth` (int, default: 1000) - QoS parameter 'keep_last'.
- `critical` (bool, default: false) - Whether state degradations should be published immediately (instead of adhering to the `pub_rate`)
- `publish_values` (bool, default: true) - Whether to fill the values dict in the status after aggregation (false saves bandwidth).

# Tutorials

TODO: Port tutorials #contributions-welcome
