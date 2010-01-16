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

##\brief Tests that robot monitor opens and doesn't throw exception

PKG = 'robot_monitor'
import roslib; roslib.load_manifest(PKG)

WXVER = '2.8'
import wxversion
if wxversion.checkInstalled(WXVER):
  wxversion.select(WXVER)
else:
  print >> sys.stderr, "This application requires wxPython version %s"%(WXVER)
  sys.exit(1)

from robot_monitor.robot_monitor_panel import RobotMonitorPanel
import wx

import threading
import unittest
import rospy, rostest
from time import sleep
import sys
from optparse import OptionParser

DURATION = 10

##\brief Main frame of robot monitor
class RobotMonitorFrame(wx.Frame):
    def __init__(self, parent, title):
        wx.Frame.__init__(self, parent, wx.ID_ANY, title)

        self.panel = RobotMonitorPanel(self)
        
    def on_exit(self, e):
        self.Close(True)

##\brief wxApp of robot monitor
class RobotMonitorApp(wx.App):
    def OnInit(self):
        self._frame = RobotMonitorFrame(None, 'Robot Monitor')
        self._frame.SetSize(wx.Size(500, 700))
        self._frame.Layout()
        #self._frame.Centre()
        self._frame.Show(True)
        return True

class RoMonRunner(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.app = RobotMonitorApp()
        
    def run(self):
        self.app.MainLoop()
        

class TestRobotMonitor(unittest.TestCase):
    def __init__(self, *args):
        super(TestRobotMonitor, self).__init__(*args)
        
        rospy.init_node('test_robot_monitor', anonymous=True)
        parser = OptionParser(usage="usage ./%prog [options]", prog="test_robot_monitor.py")

        # Option comes with rostest, will fail w/o this line
        parser.add_option('--gtest_output', action="store",
                          dest="gtest")

        options, args = parser.parse_args(rospy.myargv())

        self.runner = RoMonRunner()
        self.runner.start()
        
    def test_robot_monitor(self):
        start = rospy.get_time()
        while not rospy.is_shutdown():
            if rospy.get_time() - start > DURATION:
                break
            self.assert_(self.runner.isAlive(), "Thread running robot monitor isn't running")
            sleep(1.0)
            
        # Hack for closing the app
        try:
            wx.CallAfter(self.runner.app._frame.on_exit(None))
            self.runner.join(5)
        except:
            import traceback
            self.assert_(False, "Caught exception closing robot monitor: %s" % traceback.format_exc())

        self.assert_(not self.runner.isAlive(), "Thread running robot monitor didn't join up")
        self.assert_(not rospy.is_shutdown(), "Rospy shut down")


if __name__ == '__main__':
    rostest.run(PKG, sys.argv[0], TestRobotMonitor, sys.argv)
    
