^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package self_test
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.2 (2023-03-24)
------------------

3.1.1 (2023-03-16)
------------------
* Secretly supporting galactic (`#295 <https://github.com/ros/diagnostics/issues/295>`_)
* exporting includes (`#292 <https://github.com/ros/diagnostics/issues/292>`_)
* Maintainer update
* Contributors: Austin, Christian Henkel, Ralph Lange

3.1.0 (2023-01-26)
------------------
* Merge of foxy and humble history into rolling for future maintenance from one branch only.
* Adding READMEs to the repo (`#270 <https://github.com/ros/diagnostics/issues/270>`_)
* License fixes (`#263 <https://github.com/ros/diagnostics/issues/263>`_)
* Fix/cleanup ros1 (`#257 <https://github.com/ros/diagnostics/issues/257>`_)
* Contributors: Austin, Christian Henkel, Ralph Lange

3.0.0 (2022-06-10)
------------------
* Return the actual future from async_send_request (`#209 <https://github.com/ros/diagnostics/issues/209>`_)
* Contributors: Chris Lalancette

2.1.3 (2021-08-03)
------------------

2.1.2 (2021-03-03)
------------------

2.1.1 (2021-01-28)
------------------

2.1.0 (2021-01-12)
------------------
* Update to latest ros2 rolling. (`#177 <https://github.com/ros/diagnostics/issues/177>`_)
* Contributors: Karsten Knese
2.0.9 (2022-11-12)
------------------

2.0.8 (2021-08-03)
------------------

2.0.7 (2021-03-04)
------------------

2.0.6 (2021-01-28)
------------------

2.0.5 (2021-01-06)
------------------

2.0.4 (2020-08-05)
------------------

2.0.3 (2020-07-09)
------------------

2.0.2 (2020-06-03)
------------------
* 2.0.2
  Signed-off-by: Karsten Knese <karsten.knese@us.bosch.com>
* generate changelog
  Signed-off-by: Karsten Knese <karsten.knese@us.bosch.com>
* fix linters (`#134 <https://github.com/ros/diagnostics/issues/134>`_)
  Signed-off-by: Karsten Knese <karsten.knese@us.bosch.com>
* Contributors: Karsten Knese

2.0.1 (2020-06-03)
------------------
* fix linters (`#134 <https://github.com/ros/diagnostics/issues/134>`_)
* Contributors: Karsten Knese

2.0.0 (2019-09-03)
------------------
* Migrate self_test to ros2 (`#104 <https://github.com/ros/diagnostics/issues/104>`_)
* Make Karsten Knese maintainer for ros2 branches `#115 <https://github.com/ros/diagnostics/issues/115>`_
* Contributors: Austin, Karsten Knese

1.9.3 (2018-05-02)
------------------

1.9.2 (2017-07-15)
------------------

1.9.1 (2017-07-15)
------------------

1.9.0 (2017-04-25)
------------------

1.8.10 (2016-06-14)
-------------------

1.8.9 (2016-03-02)
------------------

1.8.8 (2015-08-06)
------------------

1.8.7 (2015-01-09)
------------------
* Upgrade to gtest 1.7.0
* Contributors: trainman419

1.8.6 (2014-12-10)
------------------

1.8.5 (2014-07-29)
------------------
* Include gtest source directly
* Contributors: trainman419

1.8.4 (2014-07-24 20:51)
------------------------
* Remove stray architechture_independent flags
  This flag should be used for package which do not contain
  architecture-specific files. Compiled binaries are such a file, and
  these packages contain them.
* Contributors: Scott K Logan

1.8.3 (2014-04-23)
------------------
* Install selftest_rostest
* Contributors: Austin Hendrix

1.8.2 (2014-04-08)
------------------
* Fix linking. All tests pass.
  Fixes `#12 <https://github.com/ros/diagnostics/issues/12>`_
* Most tests pass
* Fix private nodehandle bug
* Protect tests behind CATKIN_ENABLE_TESTING.
  Fixes `#13 <https://github.com/ros/diagnostics/issues/13>`_
* Install self test executables. Fixes `#16 <https://github.com/ros/diagnostics/issues/16>`_
* Contributors: Austin Hendrix

1.8.1 (2014-04-07)
------------------
* Add myself as maintainer
* Added ability to specify the private node handle in TestRunner
  Also now specify the selftest callback queue on only the service so that it does not apply to the entire node
* fixed test related issues in some CMakeLists
* Contributors: Austin Hendrix, Brice Rebsamen, Mitchell Wills

1.8.0 (2013-04-03)
------------------

1.7.11 (2014-07-24 20:24)
-------------------------
* Fix install rules
* Fix linking on tests
* Contributors: trainman419

1.7.10 (2013-02-22)
-------------------
* Changed package.xml version number before releasing
* Contributors: Brice Rebsamen

1.7.9 (2012-12-14)
------------------
* add missing dep to catkin
* Contributors: Dirk Thomas

1.7.8 (2012-12-06)
------------------

1.7.7 (2012-11-10)
------------------

1.7.6 (2012-11-07 23:32)
------------------------
* no library in self test
* Contributors: Vincent Rabaud

1.7.5 (2012-11-07 21:53)
------------------------
* add the missing rostest dependency
* Contributors: Vincent Rabaud

1.7.4 (2012-11-07 20:18)
------------------------

1.7.3 (2012-11-04)
------------------

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
* use the proper gtest macro
* Explicitely set selftest_rostest to be in the all target, to fix `#3178 <https://github.com/ros/diagnostics/issues/3178>`_.
* Moved failed test message to just after the test, rather than after all the tests in self_test.
* Eliminated warnings from run_selftest, and made it set its return code based on whether the test passed or not.
* Undeprecated run_selftest.
* Added a check in self test for ROS having shut down. Allows a node that is self testing to exit faster on CTRL-C.
* Unit test files
* Adding unit tests for self_test
* Removing deprecated set_status_vec from self_test package
* Added Ubuntu platform tags to manifest
* Added removeByName to the self_test example.
* Added a warning if test passes but setID was not called.
* Marked diagnostic_updater and self_test as doc reviewed.
* Tweaked examples and documentation based on doc review feedback.
* Tweaked package description.
* Updated manifest documentation and authorship.
* Added a ROS_INFO at the beginning of each test.
* Doc review of self test, dox fixes
* Removed a redundant message, and took out spurious newlines.
* Added a message at the end of the self-test.
* Took out all deprecated stuff from self_test
* Updated review status to API cleared.
* Fixed example program after rename of self_test::Sequencer
* Set a timeout of zero to callAvailable in checkTest. Renamed Sequencer to TestRunner.
* Updated links in main page.
* Corrected typo in main page.
* Added main page. Took out threading by putting the self_test service in a separate queue. Created the Sequencer class to replace the now deprecated Dispatcher class. Sequencer is non-templated and does not have an owner member.
* Made changes related to Nov 1 2009 API review.
* Replaced sleep with waitForService. Took out delay parameter, added in a max_delay parameter. Took gensrv out of the CMakeLists.txt.
* Got rid of deprecated access to ~parameters.
* Updated documentation. Made doTest private as it should be. This should not break anything unless somebody is doing something really strange.
* Making self_test package build now that ros::Node is gone.
* Added extra debugging options to self_test and diagnostic_updater. On by default for self_test, off for diagnostic_updater. When on, failing statuses will be printed to the console.
* updated self_test, diagnostic_updater, dynamic_reconfigure and wge100_camera to use new ~ namespace access method
* diagnostics 0.1 commit. Removed diagnostic_analyzer/generic_analyzer and integrated into diagnostic_aggregator.
* Fixed a sneaky bug that had slipped in during the diagnostic conversion.
  (It would have been easy to catch if Warnings were more prominent.)
* Fixes for diagnostic_msgs::KeyValue::label -> key
* Fixed bug in declaration of deprecated class.
* Deprecated old self_test and diagnostic_updater APIs.
* Lengthened delay, and made it return a saner failure message if it times out waiting to start the test.
* Changed nomenclature in driver_base. Renamed method names to use camelCase.
  Got forearm_node working with driver_base: now appears to be working well.
* robot_msgs/Diagnostic*  to diagnostic_msgs/Diagnostic* and robot_srvs/SelfTest into diagnostic_msgs too
* Added DiagnosedPublisher and HeaderlessDiagnosedPublisher to automatically
  publish diagnostics upon publication, and integrated them with the
  forearm_camera.
  Started writing an outling of the driver_base classes.
* Added a selftest_rostest node that wraps a call the self-test
  service of a node for rostest.
* Unhid the add method. Renamed the internal add so it does not cause
  conflicts.
* Fixed a bug that was causing slow startup on some nodes.
* Updated to match changes to diagnostic_updater
* Upgraded self_test to use the NodeHandle API, and to allow more general
  callback functions.
* Head and hokuyo impact tests updates
* Remove all calls to ros::fini()
* service request/response -> Request/Response
* Updated for removal of boost and log4cxx as 3rdparty packages
* Added space after class name
* roscpp API changes
  * ros::node -> ros::Node
  * ros::msg -> ros::Message
  * deprecated methods removed
  * rosconsole/rosconsole.h -> ros/console.h
  * goodbye rosthread
* Merge from josh branch... compatibility with roscpp sessions merge and cmake 2.4
* results from changing ros::Time constructor and all uses of it I can find
* Self test timeout needed to be as ros::Duration instead.
* Moving package review status from wiki to manifests
* fixing usage
* Add a demonstration of returned value label pairs in the example.
* Change value_label to label
* Adding a selftest_example.cpp which shows how to use the SelfTest class.
* Slightly refactoring self test to make usage more straightforward.
* Checking in trivial selftest_server
* Adding selftest executable into self_test package.
* Changes to make self_test quit-safe.
* Adding in self_test package to do easy self_tests inside of nodes.
* Contributors: Vincent Rabaud, blaise, blaisegassend, ehberger, gerkey, jfaustwg, jleibs, leibs, mmwise, rob_wheeler, tfoote, vrabaud, watts, wattsk
