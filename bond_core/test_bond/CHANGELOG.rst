^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_bond
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.8.3 (2018-08-17)
------------------
* Fix build issue on Windows (`#38 <https://github.com/ros/bond_core/issues/38>`_)
  * Fix build issue on Windows
  * indent for better readability
* Contributors: Johnson Shih

1.8.2 (2018-04-27)
------------------
* uuid dependency fixup (`#36 <https://github.com/ros/bond_core/issues/36>`_)
  * dont export uuid dependency as this isnt anywhere in the public api
  * fixx uuid dependency in test_bond as well
* Contributors: Mikael Arguedas

1.8.1 (2017-10-27)
------------------
* fix package.xml to comply with schema (`#30 <https://github.com/ros/bond_core/issues/30>`_)
* Contributors: Mikael Arguedas

1.8.0 (2017-07-27)
------------------
* C++ style (`#28 <https://github.com/ros/bond_core/issues/28>`_)
* switch to package format 2 (`#27 <https://github.com/ros/bond_core/issues/27>`_)
* Closer to pep8 compliance (`#25 <https://github.com/ros/bond_core/issues/25>`_)
* Python3 compatibility (`#24 <https://github.com/ros/bond_core/issues/24>`_)
* Contributors: Mikael Arguedas

1.7.19 (2017-03-27)
-------------------

1.7.18 (2016-10-24)
-------------------
* fix -isystem /usr/include build breakage in gcc6
* Contributors: Mikael Arguedas

1.7.17 (2016-03-15)
-------------------
* update maintainer
* Made code a bit more readable `#12 <https://github.com/ros/bond_core/pull/12>`_
* Contributors: Esteve Fernandez, Mikael Arguedas

1.7.16 (2014-10-30)
-------------------

1.7.15 (2014-10-28)
-------------------

1.7.14 (2014-05-08)
-------------------
* Update maintainer field
* Contributors: Esteve Fernandez, Vincent Rabaud

1.7.13 (2013-08-21)
-------------------
* update check for CATKIN_ENABLE_TESTING to work with isolated built
* check for CATKIN_ENABLE_TESTING
* Contributors: Dirk Thomas

1.7.12 (2013-06-06)
-------------------
* fix dependency on exported targets if the variable is empty for the test package
* use EXPORTED_TARGETS variable instead of explicit target names
* Contributors: Dirk Thomas

1.7.11 (2013-03-13)
-------------------

1.7.10 (2013-01-13)
-------------------

1.7.9 (2012-12-27)
------------------
* fix wrong import of bondpy
* increase timeout for test
* modified dep type of catkin
* Contributors: Dirk Thomas

1.7.8 (2012-12-13)
------------------

1.7.7 (2012-12-06)
------------------
* Updated url tags in package.xml's `#1 <https://github.com/ros/bond_core/pull/1>`_
* Contributors: William Woodall

1.7.6 (2012-10-30)
------------------
* fix catkin function order
* Contributors: Dirk Thomas

1.7.5 (2012-10-27)
------------------
* clean up package.xml files
* fixed dep to rostest
* fixed python module import
* fixed test registration in cmake
* fixed compiling tests
* Contributors: Dirk Thomas

1.7.4 (2012-10-06)
------------------

1.7.3 (2012-10-02 00:19)
------------------------

1.7.2 (2012-10-02 00:06)
------------------------
* add the missing catkin dependency
* Contributors: Vincent Rabaud

1.7.1 (2012-10-01 19:00)
------------------------

1.7.0 (2012-10-01 16:51)
------------------------
* catkinize the package and bump to 1.7.0 even though it is not tagged yet
* bondpy tests now cleanly shutdown any bonds that they create.
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4036309
* Reverting all changes that were meant to debug test failures on the build farm.
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4036308
* More testing bond on the build farm: being careful to shutdown bond instances between tests.
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4036300
* Bond: debug info about status message.  Still tracking down test errors on the build farm
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4036256
* More debug info for tracking down test failures in the build farm.
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4036255
* Changed exercise_bond.py to print more information on failure in order to debug
  test failures that only occur on the build machines.
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4036253
* Bond tester was spewing warning messages even when successful
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4036126
* Added global "bond_disable_heartbeat_timeout" parameter
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4036106
* Creating package descriptions for bondpy, bondcpp, and test_bond.
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4035354
* Renamed bondtest to test_bond (`#4435 <https://github.com/ros/bond_core/issues/4435>`_)
  --HG--
  rename : bondtest/CMakeLists.txt => test_bond/CMakeLists.txt
  rename : bondtest/Makefile => test_bond/Makefile
  rename : bondtest/mainpage.dox => test_bond/mainpage.dox
  rename : bondtest/manifest.xml => test_bond/manifest.xml
  rename : bondtest/scripts/BondSM_sm.py => test_bond/scripts/BondSM_sm.py
  rename : bondtest/scripts/tester.py => test_bond/scripts/tester.py
  rename : bondtest/srv/TestBond.srv => test_bond/srv/TestBond.srv
  rename : bondtest/test/exercise_bond.cpp => test_bond/test/exercise_bond.cpp
  rename : bondtest/test/exercise_bond.py => test_bond/test/exercise_bond.py
  rename : bondtest/test/test_callbacks_cpp.cpp => test_bond/test/test_callbacks_cpp.cpp
  rename : bondtest/test/test_callbacks_py.py => test_bond/test/test_callbacks_py.py
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4032810
* Contributors: Vincent Rabaud, sglaser
