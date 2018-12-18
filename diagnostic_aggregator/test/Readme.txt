########### READ ME ##############################
This document contains the steps to execute few tests.
1. Build the package
 # colcon build

2. Do the test
 # colcon test

3. The executable are generated follow the below steps to run tests.

3.1 source local setup 
 # source install/local_setup.sh

3.2 run the executables.
 All test cases can be run in following way.  we need to kill the nodes launch by test after test cases execution. This is bug in launch services .  
 # python3 src/diagnostics/diagnostic_aggregator/test/add_analyzers_test.py 
 # ps -ef // to find out node process id 
 # kill -9 "process id of newly started node for test case " 	
 # python3 src/diagnostics/diagnostic_aggregator/test/aggregator_test.py
 # ps -ef 
 # kill -9 "process id of newly started node for test case " 	
 # python3 src/diagnostics/diagnostic_aggregator/test/expected_stale_test.py
 # ps -ef 
 # kill -9 "process id of newly started node for test case " 	
 # python3 src/diagnostics/diagnostic_aggregator/test/multiple_match_test.py 
 # ps -ef 
 # kill -9 "process id of newly started node for test case " 	


