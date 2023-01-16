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
[INFO] [1673272679.206311476] [self_test_client]: Self test FAILED for device with id: [12345]
[INFO] [1673272679.206352683] [self_test_client]: 1) Pretest
[INFO] [1673272679.206359051] [self_test_client]: 	Pretest completed successfully.
[INFO] [1673272679.206363773] [self_test_client]: 2) ID Lookup
[INFO] [1673272679.206368112] [self_test_client]: 	ID Lookup successful
[INFO] [1673272679.206374559] [self_test_client]: 3) Exception generating test
[ERROR] [1673272679.206378739] [self_test_client]: 	Uncaught exception: we did something that threw an exception
[INFO] [1673272679.206383122] [self_test_client]: 4) Value generating test
[INFO] [1673272679.206387171] [self_test_client]: 	We successfully changed the value.
[INFO] [1673272679.206391734] [self_test_client]: 	[some value] 42
[INFO] [1673272679.206395812] [self_test_client]: 5) Value testing test
[INFO] [1673272679.206399678] [self_test_client]: 	We observed the change in value
```

# C++ API
The `TestRunner` class is the main class for self tests.
It has a method `_add` which must be used to add the specific test as callback methods.
The `TestRunner` then advertises the relevant `self_test` service and calls the aforemntioned callbacks when requested.

# Nodes
## run_selftest
This node is used to call the self test service.
