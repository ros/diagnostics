Overview

Wrapper for the diagnostic_msgs::msg::DiagnosticStatus message that
makes it easier to update

Build proccedure and testing

1. Get pacakge at local system
        1.1 # mkdir -p diagnostics/src

        1.2 # cd diagnostics/src

        1.3 # git clone git@github.com:vaibhavbhadade/diagnostics.git

        1.4 # source source /opt/ros/crystal/setup.sh



2. Build the package
 # cd ../
 # colcon build

3. Do the test
 # colcon test


Ros2 migration changes

	1> Python2.7 to Python3 migration done.

	2> directory structure changes as per ros2 python pacakges

	3> changes for creating diagnostic_msg required

