Type: diagnostic_msgs/msg/DiagnosticArray

```
std_msgs/Header header # for timestamp
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
DiagnosticStatus[] status # an array of components being reported on
	byte OK=0
	byte WARN=1
	byte ERROR=2
	byte STALE=3
	byte level
	string name
	string message
	string hardware_id
	KeyValue[] values
		string key
		string value

```

```bash
# status info
---
header:
  stamp:
    sec: 1682396036
    nanosec: 978628351
  frame_id: ''
status:
- level: "\0"
  name: 'diagnostic_simple: DemoTask'
  message: running
  hardware_id: ''
  values: []
---

# status error
---
header:
  stamp:
    sec: 1682396095
    nanosec: 590431434
  frame_id: ''
status:
- level: "\x02"
  name: 'diagnostic_simple: DemoTask'
  message: running
  hardware_id: ''
  values: []
---

# status warning
---
header:
  stamp:
    sec: 1682396159
    nanosec: 892125876
  frame_id: ''
status:
- level: "\x01"
  name: 'diagnostic_simple: DemoTask'
  message: running
  hardware_id: ''
  values: []
---

```

## Add other data to Task
```python
def run(self, stat: DiagnosticStatusWrapper):
        stat.summary(DiagnosticStatus.WARN, "running")
        stat.add("key1", "val1")
        stat.add("key2", "val2")
        return stat
```

```bash
---
header:
  stamp:
    sec: 1682396583
    nanosec: 587749252
  frame_id: ''
status:
- level: "\x01"
  name: 'diagnostic_simple: DemoTask'
  message: running
  hardware_id: ''
  values:
  - key: key1
    value: val1
  - key: key2
    value: val2
---

```