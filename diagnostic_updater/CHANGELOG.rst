^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package diagnostic_updater
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.5 (2021-01-06)
------------------

2.0.4 (2020-08-05)
------------------

2.0.3 (2020-07-09)
------------------

2.0.2 (2020-06-03)
------------------
* 2.0.2
* Re-add leading character to node name (`#109 <https://github.com/ros/diagnostics/issues/109>`_)
* Ros2 migrate diagnostic aggregator (`#118 <https://github.com/ros/diagnostics/issues/118>`_)
* Fix DiagnosedPublisher (`#135 <https://github.com/ros/diagnostics/issues/135>`_)
* fix linters (`#134 <https://github.com/ros/diagnostics/issues/134>`_)
* Contributors: DensoADAS, Karsten Knese, Scott K Logan, Arne Nordmann

2.0.1 (2020-06-03)
------------------
* Re-add leading character to node name (`#109 <https://github.com/ros/diagnostics/issues/109>`_)
* Ros2 migrate diagnostic aggregator (`#118 <https://github.com/ros/diagnostics/issues/118>`_)
* Fix DiagnosedPublisher (`#135 <https://github.com/ros/diagnostics/issues/135>`_)
* fix linters (`#134 <https://github.com/ros/diagnostics/issues/134>`_)
* Contributors: DensoADAS, Arne Nordmann, Robin Vanhove, Karsten Knese, Scott K Logan

2.0.0 (2019-09-03)
------------------
* Use rclpp timer instead of custom updater logic. (`#114 <https://github.com/ros/diagnostics/issues/114>`_)
* Use std::isfinite since it is supported on all platorms. (`#123 <https://github.com/ros/diagnostics/issues/123>`_)
* Make DiagnosticStatusWrapper no longer implicitly copyable. (`#117 <https://github.com/ros/diagnostics/issues/117>`_)
* Add virtual destructor to task vector class. (`#122 <https://github.com/ros/diagnostics/issues/122>`_)
* Support for node interfaces to allow diagnostics to be used with lifecycle nodes. (`#112 <https://github.com/ros/diagnostics/issues/112>`_)
* Spin on node in diagnostic_updater example to query parameters. (`#120 <https://github.com/ros/diagnostics/issues/120>`_)
* Set diagnostic_updater default period to 1s instead of 1ns. (`#110 <https://github.com/ros/diagnostics/issues/110>`_)
* Make Karsten Knese Maintainer for ROS2 branches `#115 <https://github.com/ros/diagnostics/issues/115>`_
* Migrate diagnostic_updater to ROS2 `#102 <https://github.com/ros/diagnostics/issues/102>`_
* Custom names for FrequencyStatus and TimeStampStatus `#86 <https://github.com/ros/diagnostics/issues/86>`_
* Make FrequencyStatus' name configurable `#84 <https://github.com/ros/diagnostics/issues/84>`_
* Contributors: Austin, Dan Rose, Ian Colwell, Karsten Knese, Nils Bussas, Scott K Logan, VaibhavBhadade

1.9.3 (2018-05-02)
------------------
* Merge pull request `#73 <https://github.com/ros/diagnostics/issues/73>`_ from tue-robotics/indigo-devel
  Add a simple Heartbeat-DiagnosticTask
* Add Python version of Heartbeat DiagnosticTask
* Add a very very simple Heartbeat DiagnosticTask
* Contributors: Austin, Loy van Beek, loy

1.9.2 (2017-07-15)
------------------

1.9.1 (2017-07-15)
------------------
* Add queue size parameters on Publishers
* Minor python updates
* Contributors: trainman419

1.9.0 (2017-04-25)
------------------
* make rostest in CMakeLists optional (`ros/rosdistro#3010 <https://github.com/ros/rosdistro/issues/3010>`_)
* Fixed bug with merge summary in status wrapper
* Contributors: Lukas Bulwahn, pAIgn10

1.8.10 (2016-06-14)
-------------------

1.8.9 (2016-03-02)
------------------

1.8.8 (2015-08-06)
------------------

1.8.7 (2015-01-09)
------------------

1.8.6 (2014-12-10)
------------------
* Add queue_size to diagnostic_updater for Python.
  cf. http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#queue_size:_publish.28.29_behavior_and_queuing
* Contributors: Mike Purvis

1.8.5 (2014-07-29)
------------------

1.8.4 (2014-07-24 20:51)
------------------------

1.8.3 (2014-04-23)
------------------
* Initialize next_time\_ properly.
  Fixes `#20 <https://github.com/ros/diagnostics/issues/20>`_
* Add failing test for fast updater
* Contributors: Austin Hendrix

1.8.2 (2014-04-08)
------------------
* Fix linking. All tests pass.
  Fixes `#12 <https://github.com/ros/diagnostics/issues/12>`_
* Most tests pass
* Fix doc reference. Fixes `#14 <https://github.com/ros/diagnostics/issues/14>`_
* Contributors: Austin Hendrix

1.8.1 (2014-04-07)
------------------
* Add myself as maintainer
* Added ability to supply a custom node name (prefix) to Updater
* Added ability to supply node handle and private node handle to Updater
* fixed exporting python API to address `#10 <https://github.com/ros/diagnostics/issues/10>`_
* fixed test related issues in some CMakeLists
* check for CATKIN_ENABLE_TESTING
* Contributors: Aero, Austin Hendrix, Brice Rebsamen, Lukas Bulwahn, Mitchell Wills

1.8.0 (2013-04-03)
------------------

1.7.11 (2014-07-24 20:24)
-------------------------
* Fix linking on tests
* support python binding of diagnostic_updater on groovy
* Contributors: Ryohei Ueda, trainman419

1.7.10 (2013-02-22)
-------------------
* Changed package.xml version number before releasing
* added missing license header
* added missing license headers
* Contributors: Aaron Blasdel, Brice Rebsamen

1.7.9 (2012-12-14)
------------------
* add missing dep to catkin
* Contributors: Dirk Thomas

1.7.8 (2012-12-06)
------------------
* missing includedirs from roscpp cause compile errors.
  diagnostic_aggregator/include/diagnostic_aggregator/status_item.h:45:21: fatal error: ros/ros.h: No such file or directory
  diagnostics/diagnostic_updater/include/diagnostic_updater/diagnostic_updater.h:42:29: fatal error: ros/node_handle.h: No such file or directory
  compilation terminated.
* Contributors: Thibault Kruse

1.7.7 (2012-11-10)
------------------

1.7.6 (2012-11-07 23:32)
------------------------

1.7.5 (2012-11-07 21:53)
------------------------

1.7.4 (2012-11-07 20:18)
------------------------

1.7.3 (2012-11-04)
------------------
* fix the non-existing xml
* Contributors: Vincent Rabaud

1.7.2 (2012-10-30 22:31)
------------------------
* fix rostest
* Contributors: Vincent Rabaud

1.7.1 (2012-10-30 15:30)
------------------------
* fix a few things after the first release
* fix a few things all over
* Contributors: Vincent Rabaud

1.7.0 (2012-10-29)
------------------
* catkinize the stack
* backport the Python API from 1.7.0
* use the proper gtest macro
* Created branch 1.7.0 and reverted corresponding changes in trunk and tag 1.7.0
  As a result branch 1.7.0 contains the new python API, and trunk corresponds to 1.6.4
* Added Python API to diagnostic_updater
* Fixing docs for frequency status parameters, `#5093 <https://github.com/ros/diagnostics/issues/5093>`_
* Remove unused (according to K. Watts) class that depends on now
  nonexistent ros::Message
* Remove unused (according to K. Watts) class that depends on now
  nonexistent ros::Message
* Fixing formatting for diagnostic updater's update_functions. `#4523 <https://github.com/ros/diagnostics/issues/4523>`_
* Adding std_msgs dependency to diagnostic_aggregator. `#4491 <https://github.com/ros/diagnostics/issues/4491>`_
* Deprecated message methods removed in diagnostics updater
* Added Ubuntu platform tags to manifest
* Corrected the version number in which removeByName was added.
* Added a removeByName method that allows a diagnostic task to be removed from a diagnostic_updater.
* Adding checks to diagnostic status wrapper to verify output from bool values
* Removed special handling of uint8 in diagnostic_status_wrapper. Uint8 isn't always bool.
* DiagnosticStatusWrapper now has bool support in add() function. `#3860 <https://github.com/ros/diagnostics/issues/3860>`_
* Marked diagnostic_updater and self_test as doc reviewed.
* Tweaked examples and documentation based on doc review feedback.
* Dox updates for diagnostic updater
* Changed error to warning level in frequency status regression test
* Removed ROS API from doxygen. Added setHardwareID method to example code.
* Fixing param name in diagnostic updater
* Reporting frequency problems as warning, not error in diagnostic_updater, `#3555 <https://github.com/ros/diagnostics/issues/3555>`_
* Took out all deprecated stuff from diagnostic_updater.
* Made diagnostic_updater example go into bin directory.
* Changed getParam to getParamCached.
* Updated review status to API cleared.
* Returned check of diagnostic_period to only happen when the update happens pending fix of ROS 0.0, -0.0, -0.0, 0.11215413361787796, -0.0)
* Finished example and documentation. Renamed CombinationDiagnosticUpdater to CompositeDiagnosticUpdater.
* Added setHardwareID to diagnostic_updater.
* Reintroduced an Updater constructor that takes a node handle because a lot of nodes actually depend on it.
* Bug slipped into previous checkin.
* Updating documentation. Took NodeHandle parameter out of Updater constructor.
* Added setHardwareID method, and now warns if it is not used.
* Got rid of ComposableDiagnosticTask. Now all tasks are composable.
* Modified diagnostic_period so that it gets checked every time the update method is called. This way a long period can get shortened without waiting for the long period to expire.
* When a diagnostic task is first added to a diagnostic_updater, the initial status is now OK instead of error.
* Added timestamp to diagnostic updater publish call. Auto-filling of timestamps is deprecated in ROS 0.10
* Fixed spurious newline in string that was preventing compilation of diagnostic_updater users.
* Finished updating the diagnostics for diagnostic_updater.
* Commented the DiagnosedPublisher classes.
* Added some comments to diagnostic_updater and made ComposableDiagnosticTask::split_run protected.
* Updated diagnosed publisher code to be able to work with a CameraPublisher
* Getting diagnostic_updater to compile.  Still spewing a bunch of warnings.
* Added extra debugging options to self_test and diagnostic_updater. On by default for self_test, off for diagnostic_updater. When on, failing statuses will be printed to the console.
* updated self_test, diagnostic_updater, dynamic_reconfigure and wge100_camera to use new ~ namespace access method
* Corrected diagnostic status merge logic.
* diagnostics 0.1 commit. Removed diagnostic_analyzer/generic_analyzer and integrated into diagnostic_aggregator.
* Add a method to clear the DiagnosticStatus values in DiagnosticStatusWrapper.
  Clear old values when reusing DiagnosticStatusWrapper.
* Took out adds and addsf from diagnostic_updater/DiagnosticStatusWrapper now that all other nodes
  have been modified.
* Converted adds into add and add-f into addf. Left the old ones, but they
  are now deprecated.
* Updated self test for new diagnostic format.
* Updated DiagnosticStatusWrapper for changes in diagnostic format.
* Cleaned up DiagnosticStatusWrapper in response to change in diagnostic
  message.
* Fixes for diagnostic_msgs::KeyValue::label -> key
* fixing through diagnostic_updater
* Changed DiagnosticMessage to DiagnosticArray
* Changed DiagnosticValue to KeyValue
* Fixed bug in declaration of deprecated class.
* Deprecated old self_test and diagnostic_updater APIs.
* Minor improvements to diagnostic updater.
* Allowed Publisher to be changed in a DiagnosedPublisher. This allows the
  Publisher to be created later than the DiagnosedPublisher.
* add cstdio include for gcc 4.4
* Took out const_cast that became unnecessary thanks to the resolution of
  ticket `#1228 <https://github.com/ros/diagnostics/issues/1228>`_.
* Added missing includes.
* robot_msgs/Diagnostic*  to diagnostic_msgs/Diagnostic* and robot_srvs/SelfTest into diagnostic_msgs too
* Added DiagnosedPublisher and HeaderlessDiagnosedPublisher to automatically
  publish diagnostics upon publication, and integrated them with the
  forearm_camera.
  Started writing an outling of the driver_base classes.
* Committing change from Blaise's tree
* Added a formatted summary method to DiagnosticStatusWrapper.
* Corrected a bug in the frequency updater, and made it and the timestamp
  updaters thread safe.
* Added a TimeStampStatus diagnostic to monitor that timestamps are
  reasonably close to now.
* Slowed timing by 10x in test case to improve odds of passing on 64 bit
  architectures.
* Corrected some bugs that could have caused undefined behavior.
  Added support for automatically publishing a "Starting up" message before
  the while the node is initializing.
  Did some refactoring.
* Fixed a bug in how function classes were being added to the Updater.
* Fixed a bug in frequency diagnostic reporting.
* Modified update functions so that they are function classes. Added a
  correspondence convenience add method to Updater_base.
* Corrected a possibly infinite recursion in adds.
* Small fix to compatibility layer for old-style nodes
* Upgraded the diagnostic_updater to use NodeHandles, and to allow more
  general functions to be used.
  Started adding update_functions to do common diagnostic publishing tasks.
  This will be populated more later.
* Added a DiagnosticStatusWrapper class derived from DiagnosticStatus. It adds a few methods to more
  conveniently set the DiagnosticStatus's fields. The diagnostic_updater has been updated so that it can work
  with DiagnosticStatus or DiagnosticStatusWrapper.:
* diagnostic_updater: Now can be used with classes that don't inherit from Node.
* roscpp API changes
  * ros::node -> ros::Node
  * ros::msg -> ros::Message
  * deprecated methods removed
  * rosconsole/rosconsole.h -> ros/console.h
  * goodbye rosthread
* bogus dependency
* results from changing ros::Time constructor and all uses of it I can find
* Adding node name into diagnostic updater status names.
* Moving package review status from wiki to manifests
* Changing form of advertise in diagnostic updater.
* Removing old printf from diagnostic_updater.
* Checking in package for easy diagnostic updating.
* Contributors: Vincent Rabaud, blaise, blaisegassend, bricerebsamen, ehberger, gerkey, jfaustwg, jleibs, leibs, morgan_quigley, pmihelich, rob_wheeler, straszheim, tfoote, vrabaud, watts, wattsk
