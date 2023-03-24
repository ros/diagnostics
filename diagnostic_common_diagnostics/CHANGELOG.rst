^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package diagnostic_common_diagnostics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.2 (2023-03-24)
------------------
* replacing ntpdate with ntplib (`#289 <https://github.com/ros/diagnostics/issues/289>`_)
* Contributors: Christian Henkel

3.1.1 (2023-03-16)
------------------
* Secretly supporting galactic (`#295 <https://github.com/ros/diagnostics/issues/295>`_)
* Linting additional package (`#268 <https://github.com/ros/diagnostics/issues/268>`_)
* Maintainer update
* Contributors: Austin, Christian Henkel, Ralph Lange

3.1.0 (2023-01-26)
------------------
* Adding READMEs to the repo (`#270 <https://github.com/ros/diagnostics/issues/270>`_)
* License fixes (`#263 <https://github.com/ros/diagnostics/issues/263>`_)
* Fix/cleanup ros1 (`#257 <https://github.com/ros/diagnostics/issues/257>`_)
* Port ntp_monitor to ROS2 (`#242 <https://github.com/ros/diagnostics/issues/242>`_)
* Contributors: Austin, Christian Henkel, RFRIEDM-Trimble, Ralph Lange

1.9.3 (2018-05-02)
------------------

1.9.2 (2017-07-15)
------------------
* FIX: add missing dependency
* Contributors: trainman419

1.9.1 (2017-07-15)
------------------
* Add queue size parameters on Publishers
* Minor python updates
* Added CPU percentage monitor
  CPU monitor that outputs the average CPU percentage and a percentage per
  CPU. The user can specify the warning CPU percentage. When one CPU exceeds
  this percentage, the diagnostics status is set to WARN.
* Contributors: Rein Appeldoorn, trainman419

1.9.0 (2017-04-25)
------------------
* Remove warning for missing queue size specification
* Contributors: sandeep

1.8.10 (2016-06-14)
-------------------

1.8.9 (2016-03-02)
------------------

1.8.8 (2015-08-06)
------------------

1.8.7 (2015-01-09)
------------------
* Remove libsensors node because it isn't portable.
* Contributors: trainman419

1.8.6 (2014-12-10)
------------------
* Add voltage sensor support
* Add ignore_sensors parameter
* Add license agreement
* Style cleanup and error handling
* Initial commit of libsensors based sensors_monitor
* Contributors: Mitchell Wills, trainman419

1.8.5 (2014-07-29)
------------------

1.8.4 (2014-07-24 20:51)
------------------------

1.8.3 (2014-04-23)
------------------
* ntp_diagnostic now publishing more frequently to avoid stale
* added install rules for the common diagnostics scripts
* Contributors: Brice Rebsamen

1.8.2 (2014-04-08)
------------------

1.8.1 (2014-04-07)
------------------
* Add myself as maintainer
* fixed exporting python API to address `#10 <https://github.com/ros/diagnostics/issues/10>`_
* Contributors: Austin Hendrix, Brice Rebsamen

1.8.0 (2013-04-03)
------------------

1.7.11 (2014-07-24 20:24)
-------------------------

1.7.10 (2013-02-22)
-------------------
* Changed package.xml version number before releasing
* added missing license header
* added missing license header
* Contributors: Aaron Blasdel, Brice Rebsamen

1.7.9 (2012-12-14)
------------------
* add missing dep to catkin
* updated setup.py
* Contributors: Dirk Thomas

1.7.8 (2012-12-06)
------------------
* fix setup.py requires
* Contributors: Dirk Thomas

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
* backport the Python API from 1.7.0
* Contributors: Vincent Rabaud
