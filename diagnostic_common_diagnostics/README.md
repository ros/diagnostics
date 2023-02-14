General information about this repository, including legal information, build instructions and known issues/limitations, are given in [README.md](../README.md) in the repository root.

# The diagnostic_common_diagnostics package
This package provides generic nodes to monitor a Linux host.

Currently only the NTP monitor is ported to ROS2.

# Nodes

## ntp_monitor.py
Runs 'ntpdate' to check if the system clock is synchronized with the NTP server. 
* If the offset is smaller than `offset-tolerance`, an `OK` status will be published.
* If the offset is larger than the configured `offset-tolerance`, a `WARN` status will be published,
* if it is bigger than `error-offset-tolerance`, an `ERROR` status will be published.
* If there was an error running `ntpdate`, an `ERROR` status will be published.

### Published Topics
#### /diagnostics
diagnostic_msgs/DiagnosticArray
The diagnostics information.

### Parameters
#### ntp_hostname 
(default: "pool.ntp.org")
Hostname of NTP server.

#### offset-tolerance"
(default: 500)
Allowed offset from NTP host. Above this is a warning.

#### error-offset-tolerance
(default: 5000000)
If the offset from the NTP host exceeds this value, it is reported as an error instead of warning.

#### self_offset-tolerance
(default: 500)
Offset from self

#### diag-hostname
Computer name in diagnostics output (ex: 'c1')

#### no-self-test
(default: True)
Disable self test.

## hd_monitor.py
**To be ported**

## cpu_monitor.py
**To be ported**

## ram_monitor.py
**To be ported**

## sensors_monitor.py
**To be ported**

## tf_monitor.py
**To be ported**