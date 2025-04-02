# ROB Topic Monitor

This is a generic topic monitor to check that topic publication rate is as expected, reporting status using diagnostics.

## Acknowledgments

Originally developed for TOP-100 "BAUTIRO".

## Technical Debt

This code was originally developed for ROS 2 "Galactic" and contains backports of some functionality that are no longer necessary for later ROS 2 versions.

Moreover, the header_topic_monitor is largely copy-pasted.

## Usage

The nodes and components use the ROS 2 life-cycle based, which means they won't just activate when run. See `launch/topic_monitor_launch.py` for an example of how to start the nodes.

### Nodes

There are two nodes which behave almost the same
  * topic_monitor_node -- this supports _any_ kind of topic and checks for frequency based on receive time
  * header_topic_monitor_node -- this supports only topics with a Header, and it checks that message age is within a given range

It is totally possible to run both of these at the same time _for the same topics_, to check for both frequency
and max message age at the same time.

Some of the parameters are common for both nodes:

#### Limiting which topics to monitor

By default, the following internal topic names will be ignored, as they are usually irrelevant:

* ^/rosout$
* .*/parameter_events
* ^/diagnostics$
* .*/transition_event

`monitor_configured_only`
 If false (the default), the topic monitor will monitor everything but the internal topics mentioned above.
 To instead only monitor those topics which have been explicitly configured, set this to "true"

#### Influencing the output

`diag_prefix`
 This string will be prefixed to the name of the diagnostic for use within the aggregator.

### `topic_monitor_node`

Without any parameters, this node (component also available) will subscribe to all normal topics in a system and minimally monitor that there is activity on them (defined as "at least one message per second"). For more specific configuration, the following parameters are available.

#### Frequency Monitoring

Usually, we want to make sure that the frequency of a topic is within certain bounds.

For example, to monitor that the frequency of the "talker" topic is about 25Hz and the frequency of the "cmd_vel" topic is about 10Hz, use the following three parameters and pass them lists in the same order. This means, the second element in "min_freqs" applies to the topic given by the second element of the "topics" parameter.

    topics: [ "talker", "cmd_vel" ]
    min_freqs: [ 23.0, 8.0 ]
    max_freqs: [ 26.5, 11.0 ]

Please note that the frequencies will be computed purely based on _arrival_ timing, the header is currently ignored.

### `header_topic_monitor_node`

Without any parameters, this node (component also available) will subscribe to all normal topics in a system and minimally monitor that there is activity on them (defined as "at least one message per second"). For more specific configuration, the following parameters are available.

#### Message Age Monitoring

Usually, we want to make sure that message is not much older than expected (allowing for some jitter due to network transmission),
but also not newer than expected (which would indicate issues with timestamp or system time between machines).

Since this relies on a sender-side timestamp, this monitor only works with message that include a Header. 

NOTE: You can configure it for any topic and it _will_ interpret the data in there as a timestamp, but unless it
really is a header, the results will be rather random ;-)

Configuration is similar to the frequency monitor, but giving a range for delays instead:

    topics: [ "talker", "cmd_vel" ]
    min_delays: [ 80, 8.0 ]
    max_delays: [ 120, 11.0 ]
