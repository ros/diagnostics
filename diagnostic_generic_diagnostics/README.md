# Summary

## Node

- topic_monitor: Supervises specific topics and provides diagnostics based on thier frequency and timestamp.

![Screenshot from 2020-05-02 17-28-22](https://user-images.githubusercontent.com/22934528/80859281-5ceb5400-8c9a-11ea-8e3d-a2ac3a86d7b5.png)

- bool_monitor: Subscribes std_msgs/Bool topics and provides diagnostics based on their values.

![Screenshot from 2020-05-04 15-27-33](https://user-images.githubusercontent.com/22934528/80940868-e1b1ab80-8e1b-11ea-98fd-a1d3937b5054.png)

## Usage

### topic_monitor

Sample launch file is given as launch/topic_monitor_test.launch.

```xml
<launch>
  <node pkg="diagnostic_generic_diagnostics" type="topic_monitor" name="test_monitor" output="screen">
    <rosparam command="load" file="$(find diagnostic_generic_diagnostics)/params/topic_monitor_test.yaml"/>
  </node>
</launch>
```

### bool_monitor

Sample launch file is given as launch/bool_monitor_test.launch.

```xml
<launch>
  <node pkg="diagnostic_generic_diagnostics" type="boo_monitor" name="test_monitor" output="screen">
    <rosparam command="load" file="$(find diagnostic_generic_diagnostics)/params/boo_monitor_test.yaml"/>
  </node>
</launch>
```

## Params

### params for topic_monitor

Sample parameter file is given as params/topic_monitor_test.yaml

- topic (string): Name of the topic that the monitor node should listen to. Required.
- hardware_id (string): Arbitrary hardware id which will be displayed in diagnostic message. Optional. Default: "".
- custom_fields (array): Array of params for fields which will be added to diagnostic message. Optional. Default: blank arary.
  - key (string): Key of the field newly added.
  - value (string): Value of the field newly added. Only string is supported. Formatted string is not supported.
  - level (int): Indicates when the field will be displayed. Sum of the OK(1), WARN(2) and ERROR(4). See pic for more detail.
- headerless (bool): If true, the monitor node does not watch topic's timestamp. Optional. Default false.
- max_freq (double): Upper bound of desired frequency [Hz].
- min_freq (double): Lower bound of desired frequency [Hz].
- tolerance (double): Determines width of margin before the diagnostics outputs error state.
- window_size (int): Lendth of the window for moving average calculation [sec].

![diag P1](https://user-images.githubusercontent.com/22934528/80859252-1eee3000-8c9a-11ea-9bf2-1eb89134d215.png)

```yaml
topics:
  - topic: /test/hoge
    hardware_id: hoge-hw
    custom_fields:
      - key: hoge-key
        value: hoge-val
        level: 1
      - key: hogehoge-key
        value: hogehoge-val
        level: 2
      - key: hogefuga-key
        value: hogefuga-val
        level: 3
    headerless: false
    max_freq: 15.0
    min_freq: 15.0
    tolerance: 0.20
    window_size: 10
  - topic: /test/fuga
    hardware_id: fuga-hw
    custom_fields:
      - key: fuga-key
        value: fuga-val
        level: 4
      - key: fugafuga-key
        value: fugafuga-val
        level: 5
      - key: fugahoge-key
        value: fugahoge-val
        level: 6
      - key: fugahogefuga-key
        value: fugahogefuga-val
        level: 7
    headerless: true
    max_freq: 60.0
    min_freq: 10.0
    tolerance: 0.20
    window_size: 5
```

### params for bool_monitor

Sample parameter file is given as params/topic_monitor_test.yaml

- topic (string): Name of the topic that the monitor node should listen to. Required.
- hardware_id (string): Arbitrary hardware id which will be displayed in diagnostic message. Optional. Default: "".
- custom_fields (array): Array of params for fields which will be added to diagnostic message. Optional. Default: blank arary.
  - key (string): Key of the field newly added.
  - value (string): Value of the field newly added. Only string is supported. Formatted string is not supported.
  - level (int): Indicates when the field will be displayed. Sum of the OK(1), WARN(2) and ERROR(4). See pic for more detail.
- publish_error (bool): If true, the monitor publishes "error". Otherwise, the monitor publishes "warn". Default true.
- timer_update (bool): If true, the monitor keeps updating /diagnostics message.
- invert (bool): Inverts True/False of input topic.

```yaml
topics:
  - topic: /test/hoge
    hardware_id: hoge-hw
    custom_fields:
      - key: hoge-key
        value: hoge-val
        level: 1
      - key: hogehoge-key
        value: hogehoge-val
        level: 2
      - key: hogefuga-key
        value: hogefuga-val
        level: 3
    publish_error: false
    invert: false
    timer_update: false
  - topic: /test/fuga
    hardware_id: fuga-hw
    custom_fields:
      - key: fuga-key
        value: fuga-val
        level: 4
      - key: fugafuga-key
        value: fugafuga-val
        level: 5
      - key: fugahoge-key
        value: fugahoge-val
        level: 6
      - key: fugahogefuga-key
        value: fugahogefuga-val
        level: 7
    publish_error: true
    invert: true
    timer_update: true
```
