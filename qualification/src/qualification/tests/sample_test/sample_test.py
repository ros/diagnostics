# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#! /usr/bin/env python

import wx
from wx import xrc
import wx.lib.plot as plot
import math

from qualification import *

from robot_msgs.msg import *

class SampleTest(BaseTest):
  def __init__(self, parent, func):
    self.parent = parent
    self.res = xrc.XmlResource(execution_path('sample_test.xrc'))
    self.panel = self.res.LoadPanel(self.parent, 'sample_test')
    BaseTest.__init__(self, parent, self.panel, func)

    BaseTest.Log(self,'Starting test of device: \'Sample Test\'680102401010!')

    self.plot_panel = xrc.XRCCTRL(self.panel, 'plot_panel')
    self.plot = plot.PlotCanvas(self.panel,-1)
    NestPanel(self.plot_panel, self.plot)

    self.i = 0

    self.panel.Bind(wx.EVT_BUTTON, self.OnStop, id=xrc.XRCID('stop_button'))
    self.panel.Bind(wx.EVT_IDLE, self.OnIdle)

  def OnStop(self, evt):
    BaseTest.Log(self,'Test completed!')
    out_stat = []
    out_stat.append(DiagnosticStatus(0, 'Stat 1', 'Something passed', []))
    out_stat.append(DiagnosticStatus(1, 'Stat 2', 'Something else warned', [DiagnosticValue(42.1, 'foo')]))
    out = DiagnosticMessage(None, out_stat)
    BaseTest.Done(self, out)
    
  def OnIdle(self, evt):
    markers = plot.PolyLine(map(lambda x: (x + self.i, math.sin(x + self.i)), range(0,100)))
    gc = plot.PlotGraphics([(markers)], 'Motor Data', 'Time', 'Velocity')
    self.plot.Draw(gc)
    self.i += 1
    evt.RequestMore(True)
