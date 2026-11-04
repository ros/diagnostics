# Diagnostic Topic Monitor

Offers diagnostic tools which monitor topics for health, such as frequency and age of messages. The output can be further processed with the [`diagnostic_aggregator`](https://github.com/ros/diagnostics/tree/ros2/diagnostic_aggregator) for cleaner observation.

## Monitors

There are two monitors currently available, which can be used as components or standalone nodes:
  * topic_frequency_monitor -- this supports _any_ kind of topic and checks for frequency based on receive time
  * topic_age_monitor -- this supports only topics with a Header, and it checks that message age is within a given range

It is totally possible to run both of these at the same time, for different or the same topics, to check for both frequency
and max message age at the same time.

## Usage

You typically add the monitor(s) you want to a launch file in your package and configure the parameters to your liking. However, the nodes and components are ROS 2 life-cycle based, which means you need to configure and activate them manually.

See the launch tests in the `test` folder for examples of how start the standalone monitor nodes, and example configs in `test/config`.

> TIP: If you like the brevity and don't mind the added dependency, you can use the [nav2_lifecycle_manager](https://docs.nav2.org/configuration/packages/configuring-lifecycle.html) with parameter `autostart` set to `true` and target your monitor nodes to autostart them.

> Example launch snippet:
```python
# Include this node in your LaunchConfiguration
autostart_monitors_node = LifecycleNode(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="autostart_monitors",
        parameters=[
            {
              "autostart": True,
              "node_names": [<your monitor nodes>],
              "bond_timeout": 0.0,
            }
        ],
    )
```

> For this, you need to have the `nav2_lifecycle_manager` package available (binary or source code installation), and ideally add it as an `exec_depend` in your package.

### Limiting which topics to monitor

By default, the following internal topic names will be ignored by the monitors, as they are usually irrelevant:

* ^/rosout$
* .*/parameter_events
* ^/diagnostics$
* .*/transition_event

ROS2 parameter `monitor_configured_only` for both monitors:

If false (the default), the topic monitor will monitor everything but the internal topics mentioned above.
To instead only monitor those topics which have been explicitly configured, set this to "true"

### Influencing the output

ROS2 parameter `diag_prefix` for both the monitors:

This string will be prefixed to the name of the diagnostic for use within the aggregator.

## `topic_frequency_monitor`

Without any parameters, this node (component also available) will subscribe to all normal topics in a system and minimally monitor that there is activity on them (defined as "at least one message per second"). For more specific configuration, the following parameters are available.

### Frequency Monitoring

Usually, we want to make sure that the frequency of a topic is within certain bounds.

For example, to monitor that the frequency of the "talker" topic is about 25Hz and the frequency of the "cmd_vel" topic is about 10Hz, use the following three parameters and pass them lists in the same order. This means, the second element in "min_values" applies to the topic given by the second element of the "topics" parameter.

    topics: [ "talker", "cmd_vel" ]
    min_values: [ 23.0, 8.0 ]
    max_values: [ 26.5, 11.0 ]

Please note that the frequencies will be computed purely based on _arrival_ timing, the header is currently ignored.

## `topic_age_monitor`

Without any parameters, this node (component also available) will subscribe to all normal topics in a system and minimally monitor that there is activity on them (defined as "at least one message per second"). For more specific configuration, the following parameters are available.

### Message Age Monitoring

Usually, we want to make sure that message is not much older than expected (allowing for some jitter due to network transmission),
but also not newer than expected (which would indicate issues with timestamp or system time between machines).

Since this relies on a sender-side timestamp, this monitor only works with message that include a Header.

NOTE: You can configure it for any topic and it _will_ interpret the data in there as a timestamp, but unless it
really is a header, the results will be rather random ;-)

Configuration is similar to the frequency monitor, but giving a range for delays instead:

    topics: [ "talker", "cmd_vel" ]
    min_values: [ 80, 8.0 ]
    max_values: [ 120, 11.0 ]
    monitor_configured_only: True

> It is highly prudent to only monitor configured topics in the age monitor. If you set it to monitor all,
and you have topics without header fields, you will get a lot of warning messages!!

## Testing

To run all tests in this package:

```bash
colcon test --event-handlers console_direct+ --packages-select diagnostic_topic_monitor && colcon test-result
```
