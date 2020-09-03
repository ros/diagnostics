^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package diagnostic_analysis
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.10.2 (2020-09-03)
-------------------

1.10.1 (2020-08-20)
-------------------

1.10.0 (2020-08-11)
-------------------
* Make Guglielmo Gemignani ROS1 maintainer (`#155 <https://github.com/ros/diagnostics/issues/155>`_)
* Contributors: Guglielmo Gemignani

1.9.4 (2020-04-01)
------------------
* noetic release (`#136 <https://github.com/ros/diagnostics/issues/136>`_)
* Contributors: Alejandro Hernández Cordero

1.9.3 (2018-05-02)
------------------

1.9.2 (2017-07-15)
------------------

1.9.1 (2017-07-15)
------------------

1.9.0 (2017-04-25)
------------------
* Install diagnostic_analysis nodes
  Fixes `#51 <https://github.com/ros/diagnostics/issues/51>`_
* make rostest in CMakeLists optional (`ros/rosdistro#3010 <https://github.com/ros/rosdistro/issues/3010>`_)
* Contributors: Lukas Bulwahn, trainman419

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

1.8.5 (2014-07-29)
------------------

1.8.4 (2014-07-24 20:51)
------------------------

1.8.3 (2014-04-23)
------------------

1.8.2 (2014-04-08)
------------------
* Most tests pass
* Contributors: Austin Hendrix

1.8.1 (2014-04-07)
------------------
* Add myself as maintainer
* check for CATKIN_ENABLE_TESTING
* Contributors: Austin Hendrix, Lukas Bulwahn

1.8.0 (2013-04-03)
------------------

1.7.11 (2014-07-24 20:24)
-------------------------
* Fix python setup in diagnostic_analysis
* Contributors: trainman419

1.7.10 (2013-02-22)
-------------------
* removed rostest from test_depend in diagnostic_analysis
* Changed package.xml version number before releasing
* Removed duplicated test dependancies
  test_depend tags should not duplicate run/build depends tags.
* Contributors: Aaron Blasdel, Brice Rebsamen

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

1.7.5 (2012-11-07 21:53)
------------------------

1.7.4 (2012-11-07 20:18)
------------------------

1.7.3 (2012-11-04)
------------------

1.7.2 (2012-10-30 22:31)
------------------------

1.7.1 (2012-10-30 15:30)
------------------------
* fix a few things after the first release
* Contributors: Vincent Rabaud

1.7.0 (2012-10-29)
------------------
* catkinize the stack
* `#5364 <https://github.com/ros/diagnostics/issues/5364>`_ `#5396 <https://github.com/ros/diagnostics/issues/5396>`_ remove useless rosrecord import
* diagnostic_analysis updated to use rosbag API. `#4163 <https://github.com/ros/diagnostics/issues/4163>`_
* Added Ubuntu platform tags to manifest
* diagnostic_aggregator/diagnostic_analysis doc reviewed
* small bug fixes
* Added hardware_id field to CSV output for export_csv.py
* Removing deprecated line in CMakesList.txt for diagnostics 0.3.0
* Renamed diagnostics_analysis to diagnostic_analysis, `#2700 <https://github.com/ros/diagnostics/issues/2700>`_
* Contributors: Vincent Rabaud, gerkey, kwc, watts
