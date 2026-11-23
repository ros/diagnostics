This package provides generic nodes for monitoring a linux host.

## Nodes

### hd_monitor.py

Monitor hard drive status through the hddtemp daemon. This checks each
drive's temperature using SMART, and checks that drives have not been
removed or unplugged.

#### Published Topics

- `diagnostics`
  ([diagnostic_msgs/msg/DiagnosticArray](https://docs.ros.org/en/rolling/p/diagnostic_msgs/msg/DiagnosticArray.html)):
  Hard drive diagnostic information

### ntp_monitor.py

Monitor NTP sync status between computers. The hostname to connect is
specified as the first command-line argument, and the offset tolerances
can be set with additional command-line arguments. See `rosrun
diagnostic_common_diagnostics ntp_monitor.py -h` for more options.

#### Published Topics

- `diagnostics`
  ([diagnostic_msgs/msg/DiagnosticArray](https://docs.ros.org/en/rolling/p/diagnostic_msgs/msg/DiagnosticArray.html)):
  NTP diagnostic information

### sensors_monitor.py

Monitor system sensors using the `sensors` command. This checks the
system's built-in temperature, voltage, and fan speed status, using the
system's default thresholds.

#### Published Topics

- `diagnostics`
  ([diagnostic_msgs/msg/DiagnosticArray](https://docs.ros.org/en/rolling/p/diagnostic_msgs/msg/DiagnosticArray.html)):
  Sensor diagnostic information

#### Parameters

- `~ignore_fans` (`bool`, default: `False`): Ignore fan speed warnings

### tf_monitor.py

Monitor TF status. Checks are made that TF messages are being published,
that transforms are up-to-date, that no cycles are present in the TF
status, that there is a single authority publishing each transform, and
that frames have a consistent parent.

#### Subscribed Topics

- `tf`
  ([tf/msg/tfMessage](https://docs.ros.org/en/rolling/p/tf/msg/tfMessage.html)):
  TF topic

#### Published Topics

- `diagnostics`
  ([diagnostic_msgs/msg/DiagnosticArray](https://docs.ros.org/en/rolling/p/diagnostic_msgs/msg/DiagnosticArray.html)):
  TF diagnostic information
