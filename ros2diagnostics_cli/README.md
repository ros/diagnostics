# ROS2 diagnostic cli

ROS2 cli to analysis and monitor `/diagnostics` topic
It's alternative to `diagnostic_analysis` project that not ported yet to ROS2.

The project add `diagnostics` command to ROS2 cli with there verbs.

- list
- show
- csv

### list
Monitor the current diagnostics_status message from `/diagnostics` topic group by Node name and print `diagnostics status` name

```bash
ros2 diagnostics list   
# Result
--- time: 1682528234 ---
diagnostic_simple:
- DemoTask
- DemoTask2
```

### show
Monitor `/diagnostics` topic and print the diagnostics_status data can filter by level and node/status name

```bash
ros2 diagnostics show -h
usage: ros2 diagnostics show [-h] [-1] [-f FILTER] [--verbose] [-l {info,warn,error}]

Show diagnostics status item info

options:
  -h, --help            show this help message and exit
  -1, --once            run only once
  -f FILTER, --filter FILTER
                        filter diagnostic status name
  --verbose, -v         Display more info.
  -l {info,warn,error}, --levels {info,warn,error}
                        levels to filter, can be multiple times
```

#### demo

```bash title="show all diagnostics status"
ros2 diagnostics show
# 
--- time: 1682528494 ---
diagnostic_simple: DemoTask: WARN, running
diagnostic_simple: DemoTask2: ERROR, bad
--- time: 1682528495 ---
diagnostic_simple: DemoTask: WARN, running
diagnostic_simple: DemoTask2: ERROR, bad
```

```bash title="filter by level"
ros2 diagnostics show -l error
--- time: 1682528568 ---
diagnostic_simple: DemoTask2: ERROR, bad
--- time: 1682528569 ---
diagnostic_simple: DemoTask2: ERROR, bad
--- time: 1682528570 ---
```

```bash title="filter by name"
ros2 diagnostics show -f Task2
#
--- time: 1682528688 ---
diagnostic_simple: DemoTask2: ERROR, bad
--- time: 1682528689 ---
diagnostic_simple: DemoTask2: ERROR, bad
```

```bash title="verbose usage"
ros2 diagnostics show -l warn -v
#
--- time: 1682528760 ---
diagnostic_simple: DemoTask: WARN, running
- key1=val1
- key2=val2
--- time: 1682528761 ---
diagnostic_simple: DemoTask: WARN, running
- key1=val1
- key2=val2

```

### csv
Export `/diagnostics` topic to csv file

**CSV headers**:
- time (sec)
- level
- node name
- diagnostics status name
- message
- hardware id
- values from keyvalue field (only on verbose)


```bash
ros2 diagnostics csv --help
usage: ros2 diagnostics csv [-h] [-1] [-f FILTER] [-l {info,warn,error}] [--output OUTPUT] [--verbose]

export /diagnostics message to csv file

options:
  -h, --help            show this help message and exit
  -1, --once            run only once
  -f FILTER, --filter FILTER
                        filter diagnostic status name
  -l {info,warn,error}, --levels {info,warn,error}
                        levels to filter, can be multiple times
  --output OUTPUT, -o OUTPUT
                        export file full path
  --verbose, -v         export DiagnosticStatus values filed
```

#### Demos

```bash title="simple csv file"
ros2 diagnostics csv -o /tmp/1.csv
--- time: 1682529183 ---
1682529183,WARN,diagnostic_simple,DemoTask,running,
```

```bash title="show csv file"
cat /tmp/1.csv

1682529183,WARN,diagnostic_simple,DemoTask,running,
1682529183,ERROR,diagnostic_simple,DemoTask2,bad,
```

```bash title="filter by level"
 ros2 diagnostics csv -o /tmp/1.csv -l error
```

```bash title="filter by name with regex"
ros2 diagnostics csv -o /tmp/1.csv -f Task$ -v
```

## Todo
- More tests
- Add unit test
- DEB package and install tests
- Ideas