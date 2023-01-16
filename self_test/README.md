General information about this repository, including legal information, build instructions and known issues/limitations, are given in [README.md](../README.md) in the repository root.

# The self_test package
This package can be used to implement self tests for ROS packages.

# Overview
It publishes a service for the node to call to perform the self test.
This then performs multiple user-defined checks on the node and reports the results.

# Example
The file [selftest_example.cpp](src/selftest_example.cpp) contains an example of how to use the self_test package.

When we then call `$ ros2 run self_test run_selftest` we get the following output:
```
[INFO] [...] [...]: Self test FAILED for device with id: [12345]
[INFO] [...] [...]: 1) Pretest
[INFO] [...] [...]: 	Pretest completed successfully.
[INFO] [...] [...]: 2) ID Lookup
[INFO] [...] [...]: 	ID Lookup successful
[INFO] [...] [...]: 3) Exception generating test
[ERROR] [...] [...]: 	Uncaught exception: we did something that threw an exception
[INFO] [...] [...]: 4) Value generating test
[INFO] [...] [...]: 	We successfully changed the value.
[INFO] [...] [...]: 	[some value] 42
[INFO] [...] [...]: 5) Value testing test
[INFO] [...] [...]: 	We observed the change in value
```

# C++ API
The `TestRunner` class is the main class for self tests.
It has a method `_add` which must be used to add the specific test as callback methods.
The `TestRunner` then advertises the relevant `self_test` service and calls the aforemntioned callbacks when requested.

# Nodes
## run_selftest
This node is used to call the self test service.
