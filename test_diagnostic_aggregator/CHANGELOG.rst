^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_diagnostic_aggregator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.9.3 (2018-05-02)
------------------

1.9.2 (2017-07-15)
------------------

1.9.1 (2017-07-15)
------------------
* Add queue size parameters on Publishers
* Contributors: trainman419

1.9.0 (2017-04-25)
------------------
* Fix cmake warnings
* Changed all deprecated PLUGINLIB_DECLARE_CLASS to PLUGINLIB_EXPORT_CLASS macros
* Contributors: Aris Synodinos, trainman419

1.8.10 (2016-06-14)
-------------------

1.8.9 (2016-03-02)
------------------
* Add version dependencies in package.xml
* Contributors: trainman419

1.8.8 (2015-08-06)
------------------

1.8.7 (2015-01-09)
------------------

1.8.6 (2014-12-10)
------------------

1.8.5 (2014-07-29)
------------------

1.8.4 (2014-07-24 20:51)
------------------------

1.8.3 (2014-04-23)
------------------

1.8.2 (2014-04-08)
------------------
* No more roslib
* Fix linking
* Protect tests behind CATKIN_ENABLE_TESTING.
  Fixes `#13 <https://github.com/ros/diagnostics/issues/13>`_
* Contributors: Austin Hendrix

1.8.1 (2014-04-07)
------------------
* Add myself as maintainer
* Contributors: Austin Hendrix

1.8.0 (2013-04-03)
------------------

1.7.11 (2014-07-24 20:24)
-------------------------
* catkin libraries are linked
* Added rostest to build requirements
  Not sure how this one slipped through, but rostest is find_package()'d in the CMakeLists.txt.
* Contributors: Mehmet Murat Sevim, Scott K Logan

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
* fix issue `#3 <https://github.com/ros/diagnostics/issues/3>`_
* Contributors: Vincent Rabaud

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
* All analyzers now load using package name, except for test case. `#4117 <https://github.com/ros/diagnostics/issues/4117>`_
* Analyzer load test verifies that both old and new analyzer specification loads. `#4117 <https://github.com/ros/diagnostics/issues/4117>`_
* Using new pluginlib macro for Analyzer classes. `#4117 <https://github.com/ros/diagnostics/issues/4117>`_
* Analyzers that fail to initialize will fail and publish status message. `#3199 <https://github.com/ros/diagnostics/issues/3199>`_
* Added Ubuntu platform tags to manifest
* Moving test_diagnostic_aggregator to diagnostics stack in ros-pkg
* Contributors: Vincent Rabaud, gerkey, watts
