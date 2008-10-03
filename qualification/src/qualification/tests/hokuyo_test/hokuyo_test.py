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

import thread

ok_err = .005;
tgt_a  = 0
ok_a   = .01
tgt_b  = .71
ok_b   = .01

class HokuyoTest(BaseTest):
  def __init__(self, parent, serial, func):

    # Load the XRC resource
    self.res2 = xrc.XmlResource(execution_path('hokuyo_test.xrc'))
    
    # Initialize the BaseTest with these parts
    BaseTest.__init__(self, parent, serial, func, 'Hokuyo Top URG')

    self.roslaunch = None
    self.topic = None
    self.response = None

  # Load the instruction and test panels from the XRC resource
  def instruct_panel_gen(self, parent):
    return self.res2.LoadPanel(parent, 'instruct_panel')

  def test_panel_gen(self, parent):
    return self.res2.LoadPanel(parent, 'test_panel')

  # This is what runs once the instructions are read
  def Run(self):
    thread.start_new_thread(self.RunThread, (None,))

  def RunThread(self, arg):
    # Create a roslauncher
    config = roslaunch.ROSLaunchConfig()

    # Try loading the XML file
    try:
      loader = roslaunch.XmlLoader()
      loader.load(execution_path('hokuyo_test.xml'), config)
    except roslaunch.XmlParseException, e:
      wx.CallAfter(self.Cancel, 'Could not load back-end XML to launch necessary nodes.  Make sure the GUI is up to date.')
      return

    # Make sure we get a fresh master
    config.master.auto = config.master.AUTO_RESTART

    # Bring up the nodes
    self.roslaunch = roslaunch.ROSLaunchRunner(config)
    self.roslaunch.launch()

    # Wait for our self-test service to come up
    rospy.wait_for_service('hokuyo/self_test', 5)

    # Try to query the self_test service on the hokuyo
    try:
        s = rospy.ServiceProxy('hokuyo/self_test', SelfTest)
        self.response = s.call(SelfTestRequest(),60)
    except rospy.ServiceException, e:
      wx.CallAfter(self.Cancel, "Could not contact node via ROS.\nError was: %s\nMake sure GUI is built correctly: rosmake qualification" % (e))
      self.roslaunch.stop()
      return

    # If the self test passed, make the wall plot
    if self.response.passed:
      wx.CallAfter(self.SetupWallTest)
    else:
      # We are done.  Clean up roslaunch and finish
      self.roslaunch.stop()
      wx.CallAfter(self.Done, self.response)  

  def SetupWallTest(self):
    self.test_panel2     = self.test_panel_gen(self.parent)
    # Destructively replace the contents of parent with the test panel
    NestPanel(self.parent, self.test_panel2)
    
    # Initialize data and count
    self.all_xvals   = []
    self.all_yvals   = []
    self.val_count  = 0
    self.xvals = numpy.array([])
    self.yvals = numpy.array([])

    self.topic = rospy.TopicSub("scan", LaserScan, self.OnLaserScan)

    self.wall_flat = None
    self.wall_fit = None

  def OnLaserScan(self, data):
    # Create the angle and range arrays
    angles = numpy.arange(data.angle_min, data.angle_max, data.angle_increment);
    ranges = numpy.array(data.ranges)

    # Convert to cartesian

    self.xvals = -numpy.sin(angles)*ranges
    self.yvals = numpy.cos(angles)*ranges
    
    if (self.val_count < 120):
      self.all_xvals.append(self.xvals)
      self.all_yvals.append(self.yvals)
      self.val_count += 1
    elif (self.val_count == 120):
      wx.CallAfter(self.FinishWallTest)

  def FinishWallTest(self):

    # Probably don't need topic anymore in short term
    self.topic.unregister()

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
    vis_panel.Bind(wx.EVT_BUTTON, self.WallDone, id=xrc.XRCID('done_button'))
    vis_panel.Bind(wx.EVT_BUTTON, self.WallRecheck, id=xrc.XRCID('recheck_button'))

    all_xvals = reduce(numpy.append, self.all_xvals)
    all_yvals = reduce(numpy.append, self.all_yvals)

    # Find the linear best fit
    (a,b) = numpy.polyfit(all_xvals, all_yvals, 1)
    
    # Compute the error
    err = numpy.sqrt(sum(map(lambda x,y: (y-(a*x+b))**2, all_xvals, all_yvals))/ all_xvals.size)
    
    print "A: %f B: %f err: %f" % (a, b, err)

    self.wall_flat = DiagnosticStatus(2, 'Flat wall test', 'Measured std of wall is too high', [DiagnosticValue(err, 'std_dev')],[])

    if ( err < ok_err ):
      self.wall_flat.level = 0
      self.wall_flat.message = 'Wall measured as flat'
      self.Log('Wall measured as flat')
      laser_color = 'g.'
    else:
      self.Log('Measured std of wall is too high')
      laser_color = 'r.'
      
    values = [DiagnosticValue(a, 'slope'), DiagnosticValue(b, 'distance')]
    self.wall_fit = DiagnosticStatus(1, 'Wall Measurement', 'Measured wall has incorrect slope or distance', values , []);
    if ((abs(tgt_a - a) < ok_a) & (abs(tgt_b - b) < ok_b)):
      self.wall_fit.level = 0
      self.wall_fit.message = 'Wall detected with correct slope and distance'
      self.Log('Wall detected with correct slope and distance')
      line_color = 'b-'
    else:
      self.Log('Measured wall has incorrect slope or distance')
      line_color = 'r-'      

    # Plot the values and line of best fit
    axes = self.plot.get_figure().gca()
    axes.clear()
    axes.plot(self.xvals, self.yvals, laser_color)
    axes.plot([-2, 2], [-2 * a + b, 2 * a + b], line_color)
    axes.plot([-2, 2], [tgt_b, tgt_b], 'b--')
    
    # Adjust the size of the axes using size of the window (this is somewhat hackish)
    sz = self.plot.GetSize()
    xrng = 1.0
    yrng = 2.0*float(sz[1])/sz[0] * xrng;
    axes.axis([-xrng,xrng,0,yrng])
    axes.grid()
    
    self.plot.draw()
    self.plot.Show()

  def WallDone(self, evt):
    # Append failure to our diagnostic status
    self.response.status.append(self.wall_flat)
    self.response.status.append(self.wall_fit)

    self.roslaunch.stop()

    wx.CallAfter(self.Done, self.response)

  def WallRecheck(self, evt):
    # Append failure to our diagnostic status
    self.SetupWallTest()
