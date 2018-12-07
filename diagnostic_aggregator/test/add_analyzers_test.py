# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import threading
import rclpy
from rclpy.node import Node
import unittest
from unittest.mock import Mock
from diagnostic_msgs.srv import AddDiagnostics
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import pathlib
import os
from launch import LaunchDescription
from launch import LaunchService
from launch import LaunchIntrospector
from launch_ros import get_default_launch_description
from launch.substitutions import EnvironmentVariable
import launch_ros.actions.node
from time import sleep


TEST_NODE = 'test_add_analyzer'
TEST_NAMESPACE = '/my_ns'
PKG = 'diagnostic_aggregator'


class TestAddAnalyzer(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
         rclpy.init()
         cls.node = rclpy.create_node(TEST_NODE, namespace=TEST_NAMESPACE)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()


    def _assert_launch_no_errors_1(self, actions):
        ld1 = LaunchDescription(actions)
        self.ls1 = LaunchService()
        self.ls1.include_launch_description(ld1)
        self.t1 = threading.Thread(target=self.ls1.run, kwargs={'shutdown_when_idle': False})
        print("self.t1 is = ",self.t1)
        self.t1.start()

    def _assert_launch_no_errors(self, actions):
        ld = LaunchDescription(actions)
        self.ls = LaunchService()
        self.ls.include_launch_description(ld)
        self.t = threading.Thread(target=self.ls.run, kwargs={'shutdown_when_idle': False})
        print("self.t is = ",self.t)
        self.t.start()

    def test_create_subscription(self):
        self.Test_pass = False
        self.expected = {'/Primary', '/Primary/primary', '/Secondary', '/Secondary/secondary',}
        parameters_file_dir = pathlib.Path(__file__).resolve().parent
        parameters_file_path = parameters_file_dir / 'simple_analyzers_test.yaml'
        parameters_file_path_1 = parameters_file_dir / 'add_analyzers.yaml'
        os.environ['FILE_PATH'] = str(parameters_file_dir)

        node_action1 = launch_ros.actions.Node(
            package='diagnostic_aggregator', node_executable='aggregator_node', output='screen',
            parameters=[
                parameters_file_path,
                str(parameters_file_path),
                [EnvironmentVariable(name='FILE_PATH'), os.sep, 'simple_analyzers_test.yaml'],
                    ],
            )
        node_action = launch_ros.actions.Node(
            package='diagnostic_aggregator', node_executable='add_analyze_pub', output='screen',
            parameters=[
                parameters_file_path_1,
                str(parameters_file_path_1),
                [EnvironmentVariable(name='FILE_PATH'), os.sep, 'add_analyzers.yaml'],
                    ],
            
            )
        self._assert_launch_no_errors([node_action1])
        self._assert_launch_no_errors_1([node_action])
        sleep(10)
        self.node.create_subscription(DiagnosticArray,'/diagnostics_agg',self.cb_test_add_agg)
            
        while self.Test_pass ==False:
            rclpy.spin_once(self.node)
    
    def cb_test_add_agg(self,msg):
        print(msg)
        self._mutex = threading.Lock()
        self.agg_msgs = {}
        with self._mutex:
            for stat in msg.status:
                self.agg_msgs[stat.name] = stat
           
        # the new aggregator data should contain the extra paths. At this point
        # the paths are probably still in the 'Other' group because the bond
        # hasn't been fully formed
        self.ls.shutdown()
        self.ls1.shutdown()
        self.node.destroy_node()
        with self._mutex:
            agg_paths = [msg.name for name, msg in self.agg_msgs.items()]
            print(agg_paths)
            self.assertTrue(all(expected in agg_paths for expected in self.expected))

       
        for name, msg in self.agg_msgs.items():
            if name in self.expected: # should have just received messages on the analyzer
                self.assertTrue(msg.message == 'OK')
                
            agg_paths = [msg.name for name, msg in self.agg_msgs.items()]
            self.assertTrue(all(expected in agg_paths for expected in self.expected))
                
        self.Test_pass =True 
        
if __name__ == '__main__':
    unittest.main()
