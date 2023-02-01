## TEST PULL REQUEST BUILDING

# Overview
The diagnostics system collects information about hardware drivers and robot hardware to make them availaible to users and operators. 
The diagnostics system contains tools to collect and analyze this data.

The diagnostics system is build around the `/diagnostics` topic. The topic is used for `diagnostic_msgs/DiagnosticArray` messages. 
It contains information about the device names, status, and values. 

It contains the following packages:
- [`diagnostic_aggregator`](/diagnostic_aggregator/): Aggregates diagnostic messages from different sources into a single message.
- [`diagnostic_analysis`](/diagnostics/): *Not ported to ROS2 yet* **#contributions-welcome**
- [`diagnostic_common_diagnostics`](/diagnostic_common_diagnostics/): Predefined nodes for monitoring the Linux and ROS system.
- [`diagnostic_updater`](/diagnostic_updater/): Base classes to publishing custom diagnostic messages for Python and C++.
- [`self_test`](/self_test/): Tools to perform self tests on nodes.

## Collecting diagnostic data
At the points of interest, i.e. the hardware drivers, the diagnostic data is collected. 
The data must be published on the `/diagnostics` topic.
In the `diagnostic_updater` package, there are base classes to simplify the creation of diagnostic messages. 

## Aggregation
The `diagnostic_aggregator` package provides tools to aggregate diagnostic messages from different sources into a single message. It has a plugin system to define the aggregation rules.

## Visualization
Outside of this repository, there is [`rqt_robot_monitor`](https://index.ros.org/p/rqt_robot_monitor/) to visualize diagnostic messages that have been aggregated by the `diagnostic_aggregator`.

Diagnostics messages that are not aggregated can be visualized by [`rqt_runtime_monitor`](https://index.ros.org/p/rqt_runtime_monitor/).

# License
The source code is released under a [BSD 3-Clause license](LICENSE).
