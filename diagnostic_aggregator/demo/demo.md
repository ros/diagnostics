# Aggregator Demo

This is a simple demo to show the diagnostic_aggregator in action. It incolves one python script producing dummy diagnostic data ([demo.pub.py](./demo.pub.py)), and one diagnostic aggregator configuration ([demo.yaml](./demo.yaml)) that provides analyzers aggreating it. Within this folder start the two processes:

1. `python3 ./demo.pub.py`
2. `ros2 run diagnostic_aggregator aggregator_node __params:=./demo.yaml`
