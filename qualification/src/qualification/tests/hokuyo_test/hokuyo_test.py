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

import numpy
import wxmpl
import matplotlib

import rospy
import roslaunch

from qualification import *

from robot_msgs.msg import *
from robot_srvs.srv import *
from std_msgs.msg import LaserScan


class HokuyoTest(BaseTest):
  def __init__(self, parent, serial, func):

    # Load the XRC resource
    self.res2 = xrc.XmlResource(execution_path('hokuyo_test.xrc'))

    # Load the instruction and test panels from the XRC resource
    instruct_panel = self.res2.LoadPanel(parent, 'instruct_panel')
    test_panel = self.res2.LoadPanel(parent, 'test_panel')

    # Initialize the BaseTest with these parts
    BaseTest.__init__(self, parent, instruct_panel, test_panel, serial, func, 'Hokuyo Top URG')

    self.roslaunch = None
    self.topic = None
    self.response = None

  # This is what runs once the instructions are read
  def Run(self):
    worker = ThreadHelper(self.RunThread)

  def RunThread(self):
    # Create a roslauncher
    self.roslaunch = roslaunch.ROSLauncher()
    loader = roslaunch.XmlLoader()

    # Try loading the XML file
    try:
        loader.load(execution_path('hokuyo_test.xml'), self.roslaunch)
    except roslaunch.XmlParseException, e:
      wx.CallAfter(self.Cancel, 'Could not load back-end XML to launch necessary nodes.  Make sure the GUI is up to date.')
      return

    # Make sure we get a fresh master
    self.roslaunch.master.auto = self.roslaunch.master.AUTO_RESTART

    # Bring up the nodes
    self.roslaunch.prelaunch_check()
    self.roslaunch.load_parameters()
    self.roslaunch.launch_nodes()

    # Wait for our self-test service to come up
    rospy.wait_for_service('urglaser/self_test', 5)

    # Try to query the self_test service on the hokuyo
    try:
        s = rospy.ServiceProxy('urglaser/self_test', SelfTest)
        self.response = s.call(SelfTestRequest(),60)
    except rospy.ServiceException, e:
      wx.CallAfter(self.Cancel, "Could not contact node via ROS.\nError was: %s\nMake sure GUI is built correctly: rosmake qualification" % (e))
      self.roslaunch.stop()
      return

    # If the self test passed, make the wall plot
    if self.response.passed:
      wx.CallAfter(self.MakePlot)
    else:
      # We are done.  Clean up roslaunch and finish
      self.roslaunch.stop()
      wx.CallAfter(self.Done, self.response)  


  def MakePlot(self):

    # Load the visualization panel
    vis_panel = self.res2.LoadPanel(self.parent, 'vis_panel')
    plot_panel = xrc.XRCCTRL(vis_panel, 'plot_panel')

    # Configure the plot panel
    self.plot = wxmpl.PlotPanel(plot_panel,-1)
    self.plot.set_crosshairs(False)
    self.plot.set_selection(False)
    self.plot.set_zoom(False)
    NestPanel(plot_panel, self.plot)

    # Nest the visualization panel, removing the current panel
    NestPanel(self.parent, vis_panel)

    # Bind the fail button and on idle events
    vis_panel.Bind(wx.EVT_BUTTON, self.WallFail, id=xrc.XRCID('fail_button'))
    vis_panel.Bind(wx.EVT_IDLE, self.OnIdle)

    # Initialize data and count
    self.all_xvals   = []
    self.all_yvals   = []
    self.val_count  = 0
    self.xvals = numpy.array([])
    self.yvals = numpy.array([])

    # Subscribe to the scan topic
    self.topic = rospy.TopicSub("scan", LaserScan, self.OnLaserScan)


  def OnLaserScan(self, data):
    # Create the angle and range arrays
    angles = numpy.arange(data.angle_min, data.angle_max, data.angle_increment);
    ranges = numpy.array(data.ranges)

    # Convert to cartesian
    self.xvals = -numpy.sin(angles)*ranges
    self.yvals = numpy.cos(angles)*ranges

    if (val_count < 40):
      self.all_xvals.append(self.xvals)
      self.all_yvals.append(self.yvals)
    else:
      self.all_xvals[val_count % 40] = self.xvals
      self.all_xvals[val_count % 40] = self.yvals

  def WallFail(self, evt):
    # Append failure to our diagnostic status
    self.response.status.append(DiagnosticStatus(2, 'Flat wall test', 'Test aborted without flat wall found', [],[]))
    self.WallDone()

  def WallSucceed(self):
    # Append success to our diagnostic status
    self.response.status.append(DiagnosticStatus(0, 'Flat wall test', 'Wall measured approximately flat', [],[]))
    self.WallDone()

  def WallDone(self):
    # Clean up the topic and the roslaunch
    self.topic.unregister()
    self.roslaunch.stop()
    wx.CallAfter(self.Done, self.response)  

  def OnIdle(self, evt):

    all_xvals = reduce(numpy.append, self.all_xvals)
    all_yvals = reduce(numpy.append, self.all_yvals)

    # Find the linear best fit
    (a,b) = numpy.polyfit(all_xvals, all_yvals, 1)

    # Compute the error
    err = numpy.sqrt(sum(map(lambda x,y: (y-(a*x+b))**2, all_xvals, all_yvals))/ all_xvals.size)

    ok_err = .005;
    tgt_a  = 0
    ok_a   = .05
    tgt_b  = .8
    ok_b   = .005

    print "A: %f B: %f err: %f" % (a, b, err)
    
    if (self.val_count > 40 & err < .005 & abs(tgt_a - a) < ok_a & abs(tgt_b - b) < ok_b):
      self.WallSucceed()

    last_err = numpy.sqrt(sum(map(lambda x,y: (y-(a*x+b))**2, self.xvals, self.yvals))/ all_xvals.size)

    if (last_err < ok_err):
      color = 'g.'
    else:
      color = 'r.'
      
    # Plot the values and line of best fit
    axes = self.plot.get_figure().gca()
    axes.clear()
    axes.plot(self.xvals, self.yvals, color)
    axes.plot([-2, 2], [-2 * a + b, 2 * a + b], 'b-')

    # Adjust the size of the axes using size of the window (this is somewhat hackish)
    sz = self.plot.GetSize()
    xrng = 1.0
    yrng = 2.0*float(sz[1])/sz[0] * xrng;
    axes.axis([-xrng,xrng,0,yrng])
    axes.grid()
    
    self.plot.draw()
    self.plot.Show()
    
    # Clear local data
    self.data = None
    
    # Request more idle time
    evt.RequestMore(True)
