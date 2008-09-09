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

import wx
from wx import xrc

import wx.lib.plot as plot
import math

from threading import Thread

from qualification import *

from robot_msgs.msg import *
from robot_srvs.srv import *
import rospy
import roslaunch

from std_msgs.msg import LaserScan

import numpy
import wxmpl
import matplotlib

import time

class ThreadHelper(Thread):
  def __init__(self, func):
    Thread.__init__(self)
    self.func = func
    self.start()

  def __del__(self):
    print 'Thread helper is destructing'

  def run(self):
    self.func()


class HokuyoTest(BaseTest):
  def __init__(self, parent, serial, func):

    # Load the XRC resource
    self.res2 = xrc.XmlResource(execution_path('hokuyo_test.xrc'))

    # Load the instruction and test panels from the XRC resource
    instruct_panel = self.res2.LoadPanel(parent, 'instruct_panel')
    test_panel = self.res2.LoadPanel(parent, 'test_panel')

    # Initialize the BaseTest with these parts
    BaseTest.__init__(self, parent, instruct_panel, test_panel, serial, func, 'Hokuyo Top URG')

    self.rl = None
    self.t = None
    self.resp = None

  # This is what runs once the instructions are read
  def Run(self):
    worker = ThreadHelper(self.RunThread)

  def RunThread(self):
    # Create a roslauncher
    self.rl = roslaunch.ROSLauncher()
    loader = roslaunch.XmlLoader()

    # Try loading the XML file
    try:
        loader.load(execution_path('hokuyo_test.xml'), self.rl)
    except roslaunch.XmlParseException, e:
      wx.CallAfter(self.Cancel, 'Could not load back-end XML to launch necessary nodes.  Make sure the GUI is up to date.')
      return

    # Make sure we get a fresh master
    self.rl.master.auto = self.rl.master.AUTO_RESTART

    # Bring up the nodes
    self.rl.prelaunch_check()
    self.rl.load_parameters()
    self.rl.launch_nodes()

    # Wait for our self-test service to come up
    rospy.wait_for_service('urglaser/self_test', 5)

    # Try to query the self_test service on the hokuyo
    try:
        s = rospy.ServiceProxy('urglaser/self_test', SelfTest)
        self.resp = s.call(SelfTestRequest(),60)
    except rospy.ServiceException, e:
      wx.CallAfter(self.Cancel, "Could not contact node via ROS.\nError was: %s\nMake sure GUI is built correctly: rosmake qualification" % (e))
      self.rl.stop()
      return

    self.Log("Beginning stage 2 of test")

    # Bring down the nodes
    wx.CallAfter(self.MakePlot)

  def MakePlot(self):
    vis_panel = self.res2.LoadPanel(self.parent, 'vis_panel')
    plot_panel = xrc.XRCCTRL(vis_panel, 'plot_panel')

    NestPanel(self.parent, vis_panel)
    
    self.plot = wxmpl.PlotPanel(plot_panel,-1)
    self.plot.set_crosshairs(False)
    self.plot.set_selection(False)
    self.plot.set_zoom(False)
    NestPanel(plot_panel, self.plot)

    vis_panel.Bind(wx.EVT_BUTTON, self.OnFail, id=xrc.XRCID('fail_button'))
    vis_panel.Bind(wx.EVT_IDLE, self.OnIdle)

    self.data = None
    self.good_count = 0

    self.t = rospy.TopicSub("scan", LaserScan, self.OnLaserScan)
    

  def OnIdle(self, evt):
    if self.data:
      angles = numpy.arange(self.data.angle_min, self.data.angle_max, self.data.angle_increment);
      ranges = numpy.array(self.data.ranges)
      xvals = -numpy.sin(angles)*ranges
      yvals = numpy.cos(angles)*ranges

      (a,b) = numpy.polyfit(xvals, yvals, 1)

      err = numpy.sqrt(sum(map(lambda x,y: (y-(a*x+b))**2, xvals, yvals))/ ranges.size)
      print err

      if (err < .005):
        self.good_count += 1
        color = 'g.'
        if (self.good_count > 40):
          pass
      else:
        good_count = 0
        color = 'r.'

      axes = self.plot.get_figure().gca()
      axes.clear()
      axes.plot(xvals, yvals, color)
      axes.plot([-2, 2], [-2 * a + b, 2 * a + b], 'b-')

      sz = self.plot.GetSize()
      xrng = 0.5
      yrng = 2.0*float(sz[1])/sz[0] * xrng;
      axes.axis([-xrng,xrng,0,yrng])
      axes.grid()
      
      self.plot.draw()
      self.plot.Show()
      
      self.data = None

    evt.RequestMore(True)

  def OnLaserScan(self, data):
    self.data = data

  def OnFail(self, evt):
    self.resp.status.append(DiagnosticStatus(2, 'Flat wall test', 'Test aborted without flat wall found', [],[]))

    self.t.unregister()

    self.rl.stop()
    
    wx.CallAfter(self.Done, self.resp)
