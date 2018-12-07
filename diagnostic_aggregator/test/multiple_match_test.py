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

##\author Kevin Watts

##\brief Tests receipt of /diagnostics_agg from diagnostic aggregator

from __future__ import with_statement
PKG = 'diagnostic_aggregator'
TEST_NODE = 'test_multi_match'
TEST_NAMESPACE = '/my_ns'

#import roslib; roslib.load_manifest(PKG)

import unittest
import rclpy
from rclpy.node import Node
#from rclpy.parameter import Parameter
#import rospy, rostest
from time import sleep
import sys
from optparse import OptionParser
import threading
import types
import time
from rclpy.task import Future
from rclpy.task import Task
import pathlib
import os



from launch import LaunchDescription
from launch import LaunchService
from launch import LaunchIntrospector
from launch_ros import get_default_launch_description
from launch.substitutions import EnvironmentVariable
import launch_ros.actions.node



from diagnostic_msgs.msg import DiagnosticArray
DURATION = 15

go_1 = 0
go_2 = 0
prefix = ""
MULTI_NAME = 'multi'
HEADER1 = 'Header1'
HEADER2 = 'Header2'
def get_raw_name(agg_name):
    return agg_name.split('/')[-1]

def get_header_name(agg_name):
    return '/'.join(agg_name.split('/')[1:-1])

class DiagnosticItem:
    def __init__(self, msg):
        self.name = get_raw_name(msg.name)
        self.header = get_header_name(msg.name)
        self.level = msg.level
        self.message = msg.message

        self.update_time = time.time()

    def is_stale(self):
        return time.time() - self.update_time > 5

    def update(self, msg):
        self.level = msg.level
        self.message = msg.message

        self.update_time = time.time()



class TestAggregator(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
         rclpy.init()
         cls.node = rclpy.create_node(TEST_NODE, namespace=TEST_NAMESPACE)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _assert_launch_errors(self, actions):
        ld = LaunchDescription(actions)
        self.ls = LaunchService()
        self.ls.include_launch_description(ld)
        assert 0 != self.ls.run()
    def _assert_launch_no_errors_1(self, actions):
        ld1 = LaunchDescription(actions)
        self.ls1 = LaunchService()
        self.ls1.include_launch_description(ld1)
        self.t1 = threading.Thread(target=self.ls1.run, kwargs={'shutdown_when_idle': False})
        self.t1.start()
     
    def _assert_launch_no_errors(self, actions):
        ld = LaunchDescription(actions)
        self.ls = LaunchService()
        self.ls.include_launch_description(ld)
        self.t = threading.Thread(target=self.ls.run, kwargs={'shutdown_when_idle': False})
        self.t.start()
        #assert 0 == self.ls.run()

    def test_multi_match(self):    
        self._mutex = threading.Lock()
        self._starttime = time.time()
        self._multi_items = {}
        self.Test_pass = False 
        parameters_file_dir = pathlib.Path(__file__).resolve().parent
        parameters_file_path = parameters_file_dir / 'multiple_match_analyzers.yaml'
        os.environ['FILE_PATH'] = str(parameters_file_dir)

        node_action1 = launch_ros.actions.Node(
            package='diagnostic_aggregator', node_executable='aggregator_node', output='screen',
            parameters=[
                parameters_file_path,
                str(parameters_file_path),
                [EnvironmentVariable(name='FILE_PATH'), os.sep, 'multiple_match_analyzers.yaml'],
                    ],
            )
        node_action = launch_ros.actions.Node(
            package='diagnostic_aggregator', node_executable='multi_match_pub', output='screen')
        self._assert_launch_no_errors_1([node_action])
        self._assert_launch_no_errors([node_action1])
        sleep(10)
        self.sub = self.node.create_subscription(DiagnosticArray, '/diagnostics_agg', self.diag_agg_cb)

        while self.Test_pass ==False:
            print("spining")
            rclpy.spin_once(self.node)

    def diag_agg_cb(self, msg):
        with self._mutex:
            for stat in msg.status:
                if stat.name.find(MULTI_NAME) > 0:
                    self._multi_items[get_header_name(stat.name)] = DiagnosticItem(stat)
        
            assert(self._multi_items[HEADER1].name == MULTI_NAME)#, "Item name under %s didn't match %s" % (HEADER1, MULTI_NAME))
            assert(self._multi_items[HEADER2].name == MULTI_NAME)#, "Item name under %s didn't match %s" % (HEADER2, MULTI_NAME))
            self.Test_pass =True
            self.node.destroy_node()
            self.ls.shutdown()
            self.ls1.shutdown()



                           

if __name__ == '__main__':
    unittest.main()
