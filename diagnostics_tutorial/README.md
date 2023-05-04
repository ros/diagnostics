# Diagnostics mini tutorial
base on [code from here](http://docs.ros.org/en/melodic/api/diagnostic_updater/html/example_8cpp_source.html)

## DiagnosticTask
[API]()


[code example](diagnostics_tutorial/task_demo.py)

## DiagnosticStatusWrapper
[API](https://docs.ros.org/en/humble/p/diagnostic_updater/generated/classdiagnostic__updater_1_1DiagnosticStatusWrapper.html)
Wrapper for the diagnostic_msgs::msg::DiagnosticStatus message that makes it easier to update and handle

[code example](diagnostics_tutorial/status_wrapper_demo.py)

## CompositeDiagnosticTask
[API](https://docs.ros.org/en/humble/p/diagnostic_updater/generated/classdiagnostic__updater_1_1CompositeDiagnosticTask.html)
The CompositeDiagnosticTask allows multiple DiagnosticTask instances to be combined into a single task that produces a single DiagnosticStatusWrapped

[code example](diagnostics_tutorial/composite_task_demo.py)

## HeaderlessTopicDiagnostic
[API](https://docs.ros.org/en/humble/p/diagnostic_updater/generated/classdiagnostic__updater_1_1HeaderlessTopicDiagnostic.html)
A class to facilitate making diagnostics for a topic using a [FrequencyStatus](#frequencystatus). 

## FrequencyStatus
[API](https://docs.ros.org/en/humble/p/diagnostic_updater/generated/classdiagnostic__updater_1_1FrequencyStatus.html#classdiagnostic__updater_1_1FrequencyStatus)
A diagnostic task that monitors the frequency of an event.