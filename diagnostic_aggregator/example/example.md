# Aggregator Example

This is a simple example to show the diagnostic_aggregator in action. It involves one python script producing dummy diagnostic data ([example_pub.py](./example_pub.py)), and one diagnostic aggregator configuration ([example.yaml](./example.yaml)) that provides analyzers aggregating it.

Within this folder start the two processes:

1. `python3 ./example_pub.py`
2. `ros2 run diagnostic_aggregator aggregator_node __params:=./example.yaml`
