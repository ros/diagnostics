Overview

Wrapper for the diagnostic_msgs::msg::DiagnosticStatus message that
makes it easier to update


Dependency

        The diagnostic package has dependency on bond-core (ROS2 package) and some system libraries e.g. psutil
                1.1 To install bond-core package, copy the code from bond-core repository and build with diagnostic package.
                1.2 The psutil can be installed for python3 by executing command
                    # sudo -H python3 -m pip install psutil

Build proccedure and testing

        Note: If package is already build, start with testing (step #3)
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

	3. Do the test
	 	2.3 # colcon test


Ros2 migration changes

	1. Python2.7 to Python3 migration done.

	2. Directory structure changes as per ros2 python pacakges

	3. Changes for creating diagnostic_msg required

