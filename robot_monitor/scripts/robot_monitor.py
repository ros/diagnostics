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

##\brief Uses robot monitor panel to display aggregated diagnostics

##\author Kevin Watts

PKG = 'robot_monitor'

import roslib
roslib.load_manifest('robot_monitor')

import rospy

WXVER = '2.8'
import wxversion
if wxversion.checkInstalled(WXVER):
  wxversion.select(WXVER)
else:
  print >> sys.stderr, "This application requires wxPython version %s"%(WXVER)
  sys.exit(1)

from robot_monitor.robot_monitor_panel import RobotMonitorPanel
import wx


##\brief Main frame of robot monitor
class RobotMonitorFrame(wx.Frame):
    def __init__(self, parent, title):
        wx.Frame.__init__(self, parent, wx.ID_ANY, title)

        self._sizer = wx.BoxSizer(wx.VERTICAL)
        self._panel = RobotMonitorPanel(self)
        self._sizer.Add(self._panel, 1, wx.EXPAND)
        self.SetSizer(self._sizer)
        
        self._shutdown_timer = wx.Timer()
        self._shutdown_timer.Bind(wx.EVT_TIMER, self._on_shutdown_timer)
        self._shutdown_timer.Start(100)
        
    def _on_shutdown_timer(self, event):
        if (rospy.is_shutdown()):
            self.Close()

##\brief wxApp of robot monitor
class RobotMonitorApp(wx.App):
    def OnInit(self):
        rospy.init_node('robot_monitor', anonymous=True)

        wx.MessageBox("robot_monitor.py is deprecated, please use robot_monitor instead", "Deprecated", wx.OK|wx.ICON_WARNING)
        
        self._frame = RobotMonitorFrame(None, 'Robot Monitor')
        self._frame.SetSize(wx.Size(500, 700))
        self._frame.Layout()
        self._frame.Show(True)
        return True
        
        
if __name__ == '__main__':
    try:
        app = RobotMonitorApp()
        app.MainLoop()
    except KeyboardInterrupt, e:
        pass
    except rospy.exceptions.ROSInitException, e:
        print 'Failed to initialize node, master probably isn\'t up.'
    except Exception, e:
        import traceback
        traceback.print_exc()

