# Aggregator Example

This is a simple example to show the diagnostic_aggregator and add_analyzer in action. It involves one python script producing dummy diagnostic data ([example_pub.py](./example_pub.py)), one diagnostic aggregator configuration ([example_analyzers.yaml](./example_analyzers.yaml)) and one add_analyzer configuration ([example_add_analyzers.yaml](./example_add_analyzers.yaml)).

The aggregator will launch and load all the analyzers listed in ([example_analyzers.yaml](./example_analyzers.yaml)). Then the aggregator will be notified that there are additional analyzers that we also want to load in ([example_add_analyzers.yaml](./example_add_analyzers.yaml)). After this reload all analyzers will be active.

Run the example with `ros2 launch diagnostic_aggregator example.launch.py`
