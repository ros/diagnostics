^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package diagnostic_aggregator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.2 (2023-03-24)
------------------

3.1.1 (2023-03-16)
------------------
* exporting dependency on pluginlib fixes `#293 <https://github.com/ros/diagnostics/issues/293>`_ (`#294 <https://github.com/ros/diagnostics/issues/294>`_)
* Secretly supporting galactic (`#295 <https://github.com/ros/diagnostics/issues/295>`_)
* Linting additional package (`#268 <https://github.com/ros/diagnostics/issues/268>`_)
* Fix code-analyser bug
* Maintainer update
* Contributors: Austin, Christian Henkel, Ralph Lange, Tim Clephas

3.1.0 (2023-01-26)
------------------
* Merge of foxy and humble history into rolling for future maintenance from one branch only.
* Adding READMEs to the repo (`#270 <https://github.com/ros/diagnostics/issues/270>`_)
* License fixes (`#263 <https://github.com/ros/diagnostics/issues/263>`_)
* Fix/cleanup ros1 (`#257 <https://github.com/ros/diagnostics/issues/257>`_)
* Use regex to search AnalyzerGroup
* Contributors: Alberto Soragna, Austin, Christian Henkel, Keisuke Shima, Ralph Lange

3.0.0 (2022-06-10)
------------------
* Use node clock for diagnostic_aggregator and diagnostic_updater (`#210 <https://github.com/ros/diagnostics/issues/210>`_)
* Contributors: Kenji Miyake

2.1.3 (2021-08-03)
------------------

2.1.2 (2021-03-03)
------------------
* Adapt new launch file syntax. (`#190 <https://github.com/ros/diagnostics/issues/190>`_)
* Introduce history depth parameter for subscription. (`#168 <https://github.com/ros/diagnostics/issues/168>`_)
* Contributors: Karsten Knese, Ryohsuke Mitsudome

2.1.1 (2021-01-28)
------------------
* Move Aggregator publishing to timer to allow subscription callback more processing time. (`#180 <https://github.com/ros/diagnostics/issues/180>`_)
* Contributors: cdbierl

2.1.0 (2021-01-12)
------------------
* Update to latest ros2 rolling. (`#177 <https://github.com/ros/diagnostics/issues/177>`_)
* Fix installation of diagnostic aggregator example. (`#159 <https://github.com/ros/diagnostics/issues/159>`_)
* Restore alphabetical order. (`#148 <https://github.com/ros/diagnostics/issues/148>`_)
* Aggregator bugfix, tests, and nicer example. (`#147 <https://github.com/ros/diagnostics/issues/147>`_)
* Contributors: Arne Nordmann, Georg Bartels, Karsten Knese
2.0.9 (2022-11-12)
------------------

2.0.8 (2021-08-03)
------------------

2.0.7 (2021-03-04)
------------------

2.0.6 (2021-01-28)
------------------
* Move Aggregator publishing to timer to allow subscription callback more processing time. (`#179 <https://github.com/ros/diagnostics/issues/179>`_)
* Contributors: cdbierl

2.0.5 (2021-01-06)
------------------
* Set aggregator subscription history depth to 1000. (`#174 <https://github.com/ros/diagnostics/issues/174>`_)
* Contributors: cdbierl

2.0.4 (2020-08-05)
------------------
* Fix installation of diagnostic aggregator example. (`#159 <https://github.com/ros/diagnostics/issues/159>`_) (`#160 <https://github.com/ros/diagnostics/issues/160>`_)
* Contributors: Georg Bartels

2.0.3 (2020-07-09)
------------------
* restore alphabetical order (`#148 <https://github.com/ros/diagnostics/issues/148>`_) (`#150 <https://github.com/ros/diagnostics/issues/150>`_)
  Signed-off-by: Karsten Knese <karsten.knese@us.bosch.com>
* Fixes toplevel diagnostic status calculation (`#149 <https://github.com/ros/diagnostics/issues/149>`_)
  See https://github.com/ros/diagnostics/issues/146
  Signed-off-by: Arne Nordmann <arne.nordmann@de.bosch.com>
* Contributors: Arne Nordmann, Karsten Knese

2.0.2 (2020-06-03)
------------------
* 2.0.2
* Ros2 migrate diagnostic aggregator (`#118 <https://github.com/ros/diagnostics/issues/118>`_)
  Co-authored-by: Arne Nordmann <arne.nordmann@de.bosch.com>
  Co-authored-by: Robin Vanhove <1r0b1n0@gmail.com>
* Contributors: Karsten Knese, Arne Nordmann, Robin Vanhove

2.0.1 (2020-06-03)
------------------
* Ros2 migrate diagnostic aggregator (`#118 <https://github.com/ros/diagnostics/issues/118>`_)
* Contributors: Arne Nordmann, Robin Vanhove, Karsten Knese

1.9.3 (2018-05-02)
------------------
* Merge pull request `#79 <https://github.com/ros/diagnostics/issues/79>`_ from nlamprian/indigo-devel
  Fixed base_path handling
* Merge pull request `#82 <https://github.com/ros/diagnostics/issues/82>`_ from moriarty/fix-pluginlib-deprecated-headers
  [Aggregator] Fixes C++ Warnings (pluginlib)
* [Aggregator] Fixes C++ Warnings (pluginlib)
  This fixes the following warnings:
  warning: Including header <pluginlib/class_list_macros.h>
  is deprecated,include <pluginlib/class_list_macros.hpp> instead. [-Wcpp]
  warning: Including header <pluginlib/class_loader.h>
  is deprecated, include <pluginlib/class_loader.hpp> instead. [-Wcpp]
  The .hpp files have been backported to indigo
* Fixed base_path handling
* Upstream missing changes to add_analyzers
* Contributors: Alexander Moriarty, Austin, Nick Lamprianidis, trainman419

1.9.2 (2017-07-15)
------------------

1.9.1 (2017-07-15)
------------------
* Add queue size parameters on Publishers
* add_analyzers improvements
  * Warning message when bond is broken
  * Per-bond topics to avoid queue length issues
* Option to make diagnostics in Other an error
* Contributors: trainman419

1.9.0 (2017-04-25)
------------------
* Longer settling time
* Fix race condition in unload
* Fix cmake warnings
* make rostest in CMakeLists optional (`ros/rosdistro#3010 <https://github.com/ros/rosdistro/issues/3010>`_)
* Changed all deprecated PLUGINLIB_DECLARE_CLASS to PLUGINLIB_EXPORT_CLASS macros
* Contributors: Aris Synodinos, Lukas Bulwahn, trainman419

1.8.10 (2016-06-14)
-------------------
* Start bond after add_diagnostics service is available
* Contributors: Mustafa Safri

1.8.9 (2016-03-02)
------------------
* Add version dependencies in package.xml
* Add version check in cmake
* Add functionality for dynamically adding analyzers
* Contributors: Michal Staniaszek, trainman419

1.8.8 (2015-08-06)
------------------
* Fix `#17 <https://github.com/ros/diagnostics/issues/17>`_
* Contributors: trainman419

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
* Install analyzer_loader. Fixes `#24 <https://github.com/ros/diagnostics/issues/24>`_
* Add dependency on message generation
* Remove stray architechture_independent flags
  This flag should be used for package which do not contain
  architecture-specific files. Compiled binaries are such a file, and
  these packages contain them.
* Contributors: Jon Binney, Scott K Logan, trainman419

1.8.3 (2014-04-23)
------------------
* Fix stale aggregation bug
* Clean up stale check
  Fixes `#21 <https://github.com/ros/diagnostics/issues/21>`_
* Contributors: Austin Hendrix

1.8.2 (2014-04-08)
------------------
* Fix linking. All tests pass.
  Fixes `#12 <https://github.com/ros/diagnostics/issues/12>`_
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
* Install analyzer_loader
* diagnostic_aggregator) Removed redundancy in package.xml.
* Contributors: Isaac Saito, trainman419

1.7.10 (2013-02-22)
-------------------
* Changed package.xml version number before releasing
* diagnostic_aggregator) Maintainer added.
* Contributors: Brice Rebsamen, Isaac Saito

1.7.9 (2012-12-14)
------------------
* add missing dep to catkin
* Contributors: Dirk Thomas

1.7.8 (2012-12-06)
------------------
* fix issue `#1 <https://github.com/ros/diagnostics/issues/1>`_
* missing includedirs from roscpp cause compile errors.
  diagnostic_aggregator/include/diagnostic_aggregator/status_item.h:45:21: fatal error: ros/ros.h: No such file or directory
  diagnostics/diagnostic_updater/include/diagnostic_updater/diagnostic_updater.h:42:29: fatal error: ros/node_handle.h: No such file or directory
  compilation terminated.
* Contributors: Thibault Kruse, Vincent Rabaud

1.7.7 (2012-11-10)
------------------
* install missing entities
* Contributors: Vincent Rabaud

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
* fix a few things all over
* Contributors: Vincent Rabaud

1.7.0 (2012-10-29)
------------------
* catkinize the stack
* use the proper gtest macro
* fixed regression of last change in diagnostics
* added separate publisher for toplevel state in diagnostic_aggregator (`#5187 <https://github.com/ros/diagnostics/issues/5187>`_)
* Allowing analyzer_loader to build on 'all' target. WG-ROS-PKG 4935
* Error message for bad regex. `#4416 <https://github.com/ros/diagnostics/issues/4416>`_
* Fixed string literal to avoid warning
* Changed all analyzer load names to pkg/Analyzer for new pluginlib call. `#4117 <https://github.com/ros/diagnostics/issues/4117>`_
* Using new pluginlib macro for Analyzer classes. `#4117 <https://github.com/ros/diagnostics/issues/4117>`_
* Added support for taking GenericAnalyzer params as string or list in regression test. `#3199 <https://github.com/ros/diagnostics/issues/3199>`_
* StatusItem no longer prepends extra / to output name if not needed
* GenericAnalyzer doesnt report anything for num_items = 0, `#4052 <https://github.com/ros/diagnostics/issues/4052>`_
* Ignore analyzer ignores all parameters. `#3733 <https://github.com/ros/diagnostics/issues/3733>`_
* Added discard analyzer. `#3733 <https://github.com/ros/diagnostics/issues/3733>`_
* Added Ubuntu platform tags to manifest
* Fixed no items message for GenericAnalyzer. `#3199 <https://github.com/ros/diagnostics/issues/3199>`_
* rename forearm camera's on hw
* Error checking in getParamVals(). `#3846 <https://github.com/ros/diagnostics/issues/3846>`_
* Replaced boost assert with ros assert
* Aggregator now warns when message timestamp isn't set, `#3823 <https://github.com/ros/diagnostics/issues/3823>`_
* Check that we're always publishing names starting with / in diagnostic aggregator. `#3199 <https://github.com/ros/diagnostics/issues/3199>`_
* Added test for testing that diagnositc items that have been matched by >1 analyzer show up in aggregated diagnostic output. `#3840 <https://github.com/ros/diagnostics/issues/3840>`_
* AnalyzerGroup can now handle multiple analyzers matching and analyzing a single status name properly. `#3691 <https://github.com/ros/diagnostics/issues/3691>`_
* AnalyzerGroup now will have a correctly named DiagnosticStatus name if no analyzers are created. `#3807 <https://github.com/ros/diagnostics/issues/3807>`_
* Adding '/' to all output diagnostic status names, `#3743 <https://github.com/ros/diagnostics/issues/3743>`_
* Changing header message for GenericAnalyzerBase when no items found
* Correct corner case of GenericAnalyzer discarding expected items that were stale
* diagnostic_aggregator/diagnostic_analysis doc reviewed
* Tested fixes for not discarding stale items if they are expected in GenericAnalzyer, `#3616 <https://github.com/ros/diagnostics/issues/3616>`_. Needs formal regression test.
* GenericAnalyzer won't discard items if they're expected. `#3616 <https://github.com/ros/diagnostics/issues/3616>`_. Needs regression test, further verification
* Fixed a  typo.
* Corrected typo in manifest.
* Updating error message of Analyzer::match const function
* aggregator node will now catch all exceptions in aggregator, and ROS_FATAL/ROS_BREAK. This will put all exceptions to the rosconsole
* AnalyzerGroup now reports that it failed to initialize if any sub analyzers failed to initialize. AnalyzerGroup will still be able to correctly match(), analyze() and report() even if all sub-analyzers failed to initialized
* Adding Analyzer load test `#3474 <https://github.com/ros/diagnostics/issues/3474>`_
* Allowed users to set and get the level/message of a StatusItem
* Dox update for generic analyzer, other analyzer, aggregator files. Updated mainpage to get correct information
* Updated aggregator documentation in manifest
* Added documentation, warnings for incorrect initialization to diagnostic_aggregator
* Fixed Other analyzer to correctly initialize GenericAnalyzerBase
* discard_stale parameters to generic analyzer will cause it to discard any items that haven't been updated within timeout
* Corrected reporting of stale items in analyzer group
* Adding analyzer group to allow diagnostic analyzers to be grouped together. Used internally by diagnostic aggregator. `#3461 <https://github.com/ros/diagnostics/issues/3461>`_
* Remove use of deprecated rosbuild macros
* Adding xmlrpcpp back into manifest for ros-pkg `#3121 <https://github.com/ros/diagnostics/issues/3121>`_
* Adding message header, stamp in aggregator, robot/runtime monitor test scripts for ROS 0.10 compatibility
* Other analyzer will no longer report anything if no 'Other' items in diagnostic aggregator. `#3263 <https://github.com/ros/diagnostics/issues/3263>`_
* Fixing diagnostic aggregator for ROS 0.10 message header stamp change
* Fixed demo in diagnostic aggregator
* Adding all changes from API review on 11/2
* Adding all changes from API review on 11/2
* Added regex support to diagnostic aggregator, made GenericAnalyzer subclassable
* Diagnostic aggregator upgrades after 10/15 API review.
* Minor fixes before API review
* Added unit test for component analyzer to diagnostic aggregator
* Added checking or warn, error conditions to generic analyzer test
* Changes from Josh's API review
* Adding diagnostic aggregator for components, things that can be broken into sub categories. Used for motors and sensors
* Adds hasKey/getValue functions to status item, removing old toStatusMsg defn
* Fixed '/' v '\' in dox, updated demo launch file
* Forgot to make the test node a <test> for diagnostic aggregator
* Moved everything to correct class names, fixed parameter ~, and added unit test
* Renamed classes to avoid diagnostic prefix, renamed files. Removed use of ~ in param names
* Removing dependency on xmlrpc++ for `#3121 <https://github.com/ros/diagnostics/issues/3121>`_
* Changed diagnostic aggregator to use boost::shared_ptr
* Added boost linkage necessary for OS X
* Minor doc fix
* diagnostics 0.1 commit. Removed diagnostic_analyzer/generic_analyzer and integrated into diagnostic_aggregator.
* Merging the new version of pluginlib back into trunk
  r31894@att (orig r22146):  eitanme | 2009-08-18 10:30:37 -0700
  Creating a branch to work on pluginlib and get things changed
  r31896@att (orig r22148):  eitanme | 2009-08-18 10:32:35 -0700
  Starting rework... need to commit so that I can move some files around
  r31942@att (orig r22182):  eitanme | 2009-08-18 16:36:37 -0700
  Commit because Scott is moving into the office and I have to shut down my computer
  r31978@att (orig r22216):  eitanme | 2009-08-18 19:20:47 -0700
  Working on changing things over to work with the new pluginlib
  r31980@att (orig r22218):  eitanme | 2009-08-18 19:24:54 -0700
  Converted pluginlib tutorials to new pluginlib code
  r31982@att (orig r22220):  eitanme | 2009-08-18 19:28:34 -0700
  Moving joint qualification controllers over to the new pluginlib model
  r31985@att (orig r22223):  eitanme | 2009-08-18 19:40:36 -0700
  Moving people_aware_nav to new pluginlib interface
  r31986@att (orig r22224):  eitanme | 2009-08-18 19:43:09 -0700
  Moving diagnostic aggregator to the pluginlib interface
  r31987@att (orig r22225):  eitanme | 2009-08-18 19:43:51 -0700
  Moving generic analyzer to the new pluginlib interface
  r31988@att (orig r22226):  eitanme | 2009-08-18 19:44:21 -0700
  Moving carrot planner to the new pluginlib interface
  r31992@att (orig r22230):  eitanme | 2009-08-18 19:54:15 -0700
  Changing REGISTER_CLASS to PLUGINLIB_REGISTER_CLASS
  r31996@att (orig r22234):  eitanme | 2009-08-18 20:19:30 -0700
  Fixing a plugin .xml file
  r31998@att (orig r22236):  eitanme | 2009-08-18 20:25:05 -0700
  Fixing more incorrect tags
* Removing Python aggregator node, has been replaced by C++ version
* Correct function names to camelCase, added documentation
* Added C++ diagnostic_aggregator
* Display child status levels in parent status for generic analyzer
* Updated documentation, fixed copy-paste error
* diagnostic_aggregator package to filter and analyze robot diagnostics
* Contributors: Vincent Rabaud, blaise, dthomas, eitanme, gerkey, kwc, vrabaud, watts, wattsk, wheeler, wim
