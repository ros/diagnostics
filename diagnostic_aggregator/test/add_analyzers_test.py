#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import unittest
import rospy, rostest
import rosparam
import optparse
import sys
import threading
from bondpy import bondpy
from diagnostic_msgs.srv import AddDiagnostics
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

PKG = 'diagnostic_aggregator'

class TestAddAnalyzer(unittest.TestCase):
    def __init__(self, *args):
        super(TestAddAnalyzer, self).__init__(*args)
        rospy.init_node('test_add_analyzer')
        self.namespace = rospy.get_name()
        paramlist = rosparam.load_file(rospy.myargv()[1])
        # expect to receive these paths in the added analyzers
        self.expected = [paramlist[0][1] + analyzer['path'] for name, analyzer in paramlist[0][0]['analyzers'].items()]

        self._mutex = threading.Lock()
        self.agg_msgs = {}

        # put parameters in the node namespace so they can be read by the aggregator
        for params, ns in paramlist:
            rosparam.upload_params(rospy.get_name() + '/' + ns, params)

        rospy.Subscriber('/diagnostics_agg', DiagnosticArray, self.agg_cb)
        self.pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)

    def agg_cb(self, msg):
        with self._mutex:
            for stat in msg.status:
                self.agg_msgs[stat.name] = stat

    def add_analyzer(self):
        """Start a bond to the aggregator
        """
        namespace = rospy.resolve_name(rospy.get_name())
        self.bond = bondpy.Bond("/diagnostics_agg/bond" + namespace, namespace)
        self.bond.start()
        rospy.wait_for_service('/diagnostics_agg/add_diagnostics', timeout=10)
        add_diagnostics = rospy.ServiceProxy('/diagnostics_agg/add_diagnostics', AddDiagnostics)
        print(self.namespace)
        resp = add_diagnostics(load_namespace=self.namespace)
        self.assert_(resp.success, 'Service call was unsuccessful: {0}'.format(resp.message))

    def wait_for_agg(self):
        self.agg_msgs = {}
        while not self.agg_msgs and not rospy.is_shutdown():
            rospy.sleep(rospy.Duration(3))

    def test_add_agg(self):
        self.wait_for_agg()

        # confirm that the things we're going to add aren't there already
        with self._mutex:
            agg_paths = [msg.name for name, msg in self.agg_msgs.items()]
            self.assert_(not any(expected in agg_paths for expected in self.expected))
            
        # add the new groups
        self.add_analyzer()

        arr = DiagnosticArray()
        arr.header.stamp = rospy.get_rostime()
        arr.status = [
            DiagnosticStatus(name='primary', message='hello-primary'),
            DiagnosticStatus(name='secondary', message='hello-secondary')
        ]
        self.pub.publish(arr)
        self.wait_for_agg()
        # the new aggregator data should contain the extra paths. At this point
        # the paths are probably still in the 'Other' group because the bond
        # hasn't been fully formed
        with self._mutex:
            agg_paths = [msg.name for name, msg in self.agg_msgs.items()]
            self.assert_(all(expected in agg_paths for expected in self.expected))

        rospy.sleep(rospy.Duration(5)) # wait a bit for the new items to move to the right group
        arr.header.stamp = rospy.get_rostime()
        self.pub.publish(arr) # publish again to get the correct groups to show OK
        self.wait_for_agg()

        for name, msg in self.agg_msgs.items():
            if name in self.expected: # should have just received messages on the analyzer
                self.assert_(msg.message == 'OK')
                
            agg_paths = [msg.name for name, msg in self.agg_msgs.items()]
            self.assert_(all(expected in agg_paths for expected in self.expected))
                

        self.bond.shutdown()
        rospy.sleep(rospy.Duration(5)) # wait a bit for the analyzers to unload
        self.wait_for_agg()
        # the aggregator data should no longer contain the paths once the bond is shut down
        with self._mutex:
            agg_paths = [msg.name for name, msg in self.agg_msgs.items()]
            self.assert_(not any(expected in agg_paths for expected in self.expected))
        
if __name__ == '__main__':
    print('SYS ARGS:', sys.argv)
    rostest.run(PKG, sys.argv[0], TestAddAnalyzer, sys.argv)
