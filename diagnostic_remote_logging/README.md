General information about this repository, including legal information and known issues/limitations, are given in [README.md](../README.md) in the repository root.

# The diagnostic_remote_logging package

This package provides the `influx` node, which listens to diagnostic messages and integrates with InfluxDB v2 for monitoring and visualization. Specifically, it subscribes to the [`diagnostic_msgs/DiagnosticArray`](https://index.ros.org/p/diagnostic_msgs) messages on the `/diagnostics_agg` topic and the [`diagnostic_msgs/DiagnosticStatus`](https://index.ros.org/p/diagnostic_msgs) messages on the `/diagnostics_toplevel_state` topic. The node processes these messages, sending their statistics and levels to an [`InfluxDB`](http://influxdb.com) database, enabling use with tools like [`Grafana`](https://grafana.com).

As of now we only support InfluxDB v2, for support with older versions please use a proxy like [`Telegraf`](https://www.influxdata.com/time-series-platform/telegraf/). See section [Telegraf](#using-a-telegraf-proxy) for an example on how to setup.

## Node Configuration

You can send data to [`InfluxDB`](http://influxdb.com) in two ways: directly to the database or via a proxy like [`Telegraf`](https://www.influxdata.com/time-series-platform/telegraf/). While both methods are valid, using a proxy is generally recommended due to the following benefits:

- **Efficient Data Transmission**: Telegraf aggregates multiple measurements and sends them in a single request, reducing bandwidth usage and minimizing database load.
- **Enhanced Reliability**: Provides buffering in case of connection issues, ensuring no data is lost.
- **Comprehensive Metric Collection**: Telegraf can send additional system metrics (e.g., RAM, CPU, network usage) with minimal configuration.
- **Data Filtering and Transformation**: Supports preprocessing, such as filtering or transforming data, before sending it to InfluxDB.

To use either method, ensure you have a running instance of InfluxDB. The simplest way to set this up is through [`InfluxDB Cloud`](https://cloud2.influxdata.com/signup).

### Parameters

The `influx` node supports several parameters. Below is an example configuration:

```yaml
/influx:
  ros__parameters:
    connection:
      url: http://localhost:8086/api/v2/write
      token:
      bucket:
      organization:
    send:
      agg: true
      top_level_state: true
```

- `send.agg`: Enables or disables subscription to the `/diagnostics_agg` topic.
- `send.top_level_state`: Enables or disables subscription to the `/diagnostics_toplevel_state` topic.

#### InfluxDB Configuration

Set the following parameters in your configuration to match your InfluxDB instance:

- `connection.url`: The URL of your InfluxDB write API endpoint.
- `connection.token`: Your InfluxDB authentication token.
- `connection.bucket`: The target bucket in InfluxDB.
- `connection.organization`: The name of your InfluxDB organization.

### Starting the node

Afterward all configurations are set run the node with the following command:

```bash
ros2 run diagnostic_remote_logging influx --ros-args --params-file <path_to_yaml_file>
```

## Using a Telegraf Proxy

To configure Telegraf as a proxy for InfluxDB:

1. Ensure Telegraf is set up to send data to your InfluxDB instance via its configuration file (`/etc/telegraf/telegraf.conf`). Check [this link](https://docs.influxdata.com/influxdb/cloud/write-data/no-code/use-telegraf/manual-config/) for an example.
2. Add the following to the telegraf configuration file to enable the InfluxDB v2 listener:

    ```toml
    [[inputs.influxdb_v2_listener]]
      service_address = ":8086"
    ```

3. Update the `influx` node configuration to point to the appropriate URL. For example, if Telegraf is running on the same host as the `influx` node, the default `http://localhost:8086/api/v2/write` should work.

4. Leave the following parameters empty in the `influx` node configuration when using Telegraf as a proxy:

    - `connection.token`
    - `connection.bucket`
    - `connection.organization`

5. Afterwards run the node with the following command:

    ```bash
    ros2 run diagnostic_remote_logging influx --ros-args --params-file <path_to_yaml_file>
    ```
