## Overview

Using ROS tools like [rosrecord](rosrecord) and [rosplay](rosplay),
users can record and playback [bag](Bags) files containing ROS messages.
For diagnostics bag files, we use tools in the `diagnostic_analysis`
package to convert these bag files into CSV files. The `export_csv.py`
tool converts these bag files into CSV files, with a separate CSV file
for each
[diagnostics_msgs/msg/DiagnosticStatus](https://docs.ros.org/en/rolling/p/diagnostics_msgs/msg/DiagnosticStatus.html)
name. These CSV files can be analyzed with off-the-shelf spreadsheet
software.

Larger bag files can be too large for common spreadsheet software (which
support at most 65,536 rows). For large files, `sparse_csv.py` can
thin or convert these files into smaller CSV files for analysis.

### Non-Diagnostics Bagfiles

If you have any messages that aren't on the "/diagnostics" topic,
they will be discarded. 

```
Discarding message on topic: /turtle1/pose
Discarding message on topic: /turtle1/color_sensor
Discarding message on topic: /turtle1/pose
Discarding message on topic: /turtle1/color_sensor
```

 The PR2 diagnostics
toolchain uses
[diagnostic_msgs/msg/DiagnosticArray](https://docs.ros.org/en/rolling/p/diagnostic_msgs/msg/DiagnosticArray.html)
messages on the "/diagnostics" topic, which is what this package is
designed for. Only [bag files](Bags) of this type will work with the
diagnostic_analysis package. To play
other bag files, use [rostopic](https://index.ros.org/p/rostopic).

## Command-line Tools

### export_csv.py

`export_csv.py \[bag-files\]\...`

Convert bag file(s) to CSV representation. By default, this will output
the CSV file to the `output/log_filename` directory, e.g.:


```
$ rosrun diagnostic_analysis export_csv.py <bagfilename>
```



`-d OUTPUT_DIR` Change the output directory. For
<example:

```
rosrun diagnostic_analysis export_csv.py file.bag -d my_dir
```

> will store the results in the
"my_dir/output" directory.

Each "name" in the diagnostic status messages will have its own file.
This means you'll see one file for the Power Board, another for
EtherCAT Master, etc\...

### sparse_csv.py

`sparse_csv.py <csv-file> -m` (`\--max`) Shrink CSV file to a
maximum of 65,000 rows. This assists with fitting within Excel limit of
65,536 rows. This will retain every nth line as necessary to reduce the
CSV row length. For example: 

```
$ rosrun diagnostic_analysis sparse_csv.py my_bagfile.csv -m
```



`sparse_csv.py <csv-file> -l LENGTH` (`\--length`) Shrink CSV file
to specified row length by skipping every nth line as necessary.

`sparse_csv.py <csv-file> -s SKIP` (`\--skip`) Shrink CSV file by
skipping every `SKIP` lines.

## Tutorials

- [Analyzing Diagnostic Logs Off-Line](https://index.ros.org/p/diagnostics):
  This tutorial uses the
  diagnostic_analysis package to
  analyze a robot log file. These files are generated automatically by
  the robots when they're running and contain the entire diagnostics
  history of the robot. You can convert these log files into CSV files
  (comma separated value) that can be opened with any spreadsheet
  editor. This only works with
  [diagnostic_msgs/msg/DiagnosticArray](https://docs.ros.org/en/rolling/p/diagnostic_msgs/msg/DiagnosticArray.html)
  messages on the `/diagnostics` topic, used in the PR2 diagnostic
  toolchain.

## Tutorials

- [Analyzing Diagnostic Logs Off-Line](https://index.ros.org/p/diagnostics):
  This tutorial uses the
  diagnostic_analysis package to
  analyze a robot log file. These files are generated automatically by
  the robots when they're running and contain the entire diagnostics
  history of the robot. You can convert these log files into CSV files
  (comma separated value) that can be opened with any spreadsheet
  editor. This only works with
  [diagnostic_msgs/msg/DiagnosticArray](https://docs.ros.org/en/rolling/p/diagnostic_msgs/msg/DiagnosticArray.html)
  messages on the `/diagnostics` topic, used in the PR2 diagnostic
  toolchain.
