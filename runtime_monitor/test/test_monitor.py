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

##\author Kevin Watts

##\brief Tests that runtime monitor opens, closes and doesn't throw exception

PKG = 'runtime_monitor'
import roslib; roslib.load_manifest(PKG)

import wx
import unittest
import rospy, rostest
from time import sleep
import sys
from optparse import OptionParser

from runtime_monitor.monitor_panel import *

import threading

DURATION = 10

##\brief Main frame of runtime monitor for testing only
class MainWindow(wx.Frame):
    def __init__(self, parent, title):
        wx.Frame.__init__(self, parent, wx.ID_ANY, title)

        self.panel = MonitorPanel(self)
        self.SetSize(wx.Size(750, 450))

    def on_exit(self, e):
        self.Close(True)

class MonitorApp(wx.App):
    def OnInit(self):
        self._frame = MainWindow(None, 'Runtime Monitor')
        self._frame.SetSize(wx.Size(500, 700))
        self._frame.Layout()
        self._frame.Show(True)
        return True

class RuntimeRunner(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.app = MonitorApp()
        
    def run(self):
        self.app.MainLoop()
        

class TestRuntimeMonitor(unittest.TestCase):
    def __init__(self, *args):
        super(TestRuntimeMonitor, self).__init__(*args)
        
        rospy.init_node('test_runtime_monitor', anonymous=True)
        parser = OptionParser(usage="usage ./%prog [options]", prog="test_monitor.py")

        # Option comes with rostest, will fail w/o this line
        parser.add_option('--gtest_output', action="store",
                          dest="gtest")

        options, args = parser.parse_args(rospy.myargv())

        self.runner = RuntimeRunner()
        self.runner.start()
        
    def test_runtime_monitor(self):
        start = rospy.get_time()
        while not rospy.is_shutdown():
            if rospy.get_time() - start > DURATION:
                break
            self.assert_(self.runner.isAlive(), "Thread running runtime monitor isn't running.")
            sleep(1.0)
            
        try:
            wx.CallAfter(self.runner.app._frame.on_exit(None))
            self.runner.join(5)
        except:
            import traceback
            self.assert_(False, "Caught exception closing runtime monitor: %s" % traceback.format_exc())
        
        self.assert_(not self.runner.isAlive(), "Thread running runtime monitor didn't join up.")
        self.assert_(not rospy.is_shutdown(), "Rospy shut down during test.")


if __name__ == '__main__':
    rostest.run(PKG, sys.argv[0], TestRuntimeMonitor, sys.argv)
    
