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

### DiagnosedPublisher
A ROS publisher with included diagnostics. 
It diagnoses the frequency of the published messages.

