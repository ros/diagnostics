General information about this repository, including legal information, build instructions and known issues/limitations, are given in [README.md](../README.md) in the repository root.
# The diagnostic_updater package

This package is used to implement the collection of diagnostics information.

## Overview
It can for example update the state of sensors or actors of the robot.
Common tasks include
* Publish the status of a sensor topic from a device driver
* Report that a hardware device is closed
* Send an error if a value is out bounds (e.g. temperature)

## Example
The file [example.cpp](src/example.cpp) contains an example of how to use the diagnostic_updater.

## C++ and Python API
The main classes are:

### DiagnosticStatusWrapper
This class is used to create a diagnostic message. 
It simplifies the creation of the message by providing methods to set the level, name, message and values.
There is also the possibility to merge multiple DiagnosticStatusWrapper into one.

### Updater
This class is used to collect the diagnostic messages and to publish them.

When a task is added, the updater immediately publishes a placeholder status
("Node starting up", level `starting_up_status`, default `OK`).
Symmetrically, the optional `shutting_down_status` constructor argument makes the
updater publish a final status ("Node shutting down") for every task when the node
goes away, so that consumers such as `diagnostic_aggregator` see an explicit message
instead of waiting for a stale timeout.
It is disabled by default.

* C++: the final status is published automatically from a pre-shutdown callback on
  the node's context, i.e. right before `rclcpp::shutdown()` tears the context down
  (this also covers Ctrl-C). It can also be changed at runtime with
  `setShuttingDownStatus()`.
* Python: `rclpy` has no pre-shutdown hook, so `Updater.shutdown()` must be called
  explicitly before `rclpy.shutdown()` or `node.destroy_node()`.

Delivery of the final message is best effort, since the process usually exits
right after it is published.

### DiagnosedPublisher
A ROS publisher with included diagnostics. 
It diagnoses the frequency of the published messages.

## Parameters

## Parameters

- **`diagnostic_updater.period`** (double, default: `1.0`)  
  Sets the publishing period (in seconds) for the diagnostic updater. This overrides the value set in the constructor of `diagnostic_updater`.

- **`diagnostic_updater.use_fqn`** (bool, default: `false`)  
  If `true`, diagnostics are published using the fully qualified ROS node name (e.g. `/ns/node_name`) instead of `node_name`.  
