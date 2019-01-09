This file contains basic overview of diagnostics and procedure to build the package.

Overview

	The diagnostics system is designed to collect information from hardware drivers and robot hardware to users and operators for analysis, troubleshooting, and logging. The diagnostics stack contains tools for collecting, publishing, analyzing and viewing diagnostics data.

	The diagnostic package contains following sub packages
	1. diagnostic_aggregator
	2. diagnostic_analysis
	3. diagnostic_common_diagnosstics
	4. diagnostic_updator
	5. self_test

	The basic details about each sub package can be found in file named ros2_migration_readme.md present in each sub package.

Dependency

	The diagnostic package has dependency on bond-core (ROS2 package) and some system libraries e.g. psutil
		1.1 To install bond-core package, copy the code from bond-core repository and build with diagnostic package.
		1.2 The psutil can be installed for python3 by executing command
		    # sudo -H python3 -m pip install psutil

Build package

	1. Get pacakge at local system 

		1.1 # mkdir -p diagnostics/src

		1.2 # cd diagnostics/src

		1.3 # git clone git@github.com:kishornaik10/bond_core.git 
		    or git clone https://github.com/kishornaik10/bond_core.git
		    (checkout for ros2-devel branch)
		
		1.4 # git clone git@github.com:vaibhavbhadade/diagnostics.git 
		    (checkout for ros2-devel branch)

		1.5 # source /opt/ros/crystal/setup.sh 
			

	2. Build the package

		 2.1 # cd ../
		 2.2 # colcon build

	3. To run tests, Please refer ros2_migration_readme, which is present in each sub package


ROS2 Migration changes 

	The basic concept and design are same as ROS.
	All changes for migration have been done as per Migration guide.

Note

	The ros2_migration_readme is available in each sub package which contains basic overview of sub package, build and test procedure to test sub package, migration changes done with respect to migration from ROS to ROS2. The readme file also contains the limitations of package.
