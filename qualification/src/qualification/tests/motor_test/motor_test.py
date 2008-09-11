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
from mechanism_control.msg import *
from mechanism_control import mechanism
from generic_controllers.controllers import * 

class MotorTest(BaseTest):
  def __init__(self, parent, serial, func):
    self.ready=False
    self.serial = serial[0:7]
    self.count = 1
    self.finished=False
    self.testcount=1
    # Load the XRC resource
    self.res2 = xrc.XmlResource(execution_path('motor_test.xrc'))
    
    # Load the instruction and test panels from the XRC resource
    panel = 'instruct_panel_'+serial
    instruct_panel = self.res2.LoadPanel(parent, panel )
    test_panel = self.res2.LoadPanel(parent, 'test_panel')

    # Initialize the BaseTest with these parts
    BaseTest.__init__(self, parent, instruct_panel, test_panel, serial, func, 'Motor Test')

    self.roslaunch = None
    self.topic = None
    self.response = DiagnosticMessage(None,[])

  # This is what runs once the instructions are read
  def Run(self):
    worker = ThreadHelper(self.RunThread)

  def RunThread(self):
    #Create a roslauncher
    self.roslaunch = roslaunch.ROSLauncher()
    loader = roslaunch.XmlLoader()

    # Try loading the XML file
    try:
        loader.load(execution_path('xml/motor_test.xml'), self.roslaunch)
    except roslaunch.XmlParseException, e:
        wx.CallAfter(self.Cancel, 'Could not load back-end XML to launch necessary nodes.  Make sure the GUI is up to date.')
        return

    # Make sure we get a fresh master
    self.roslaunch.master.auto = self.roslaunch.master.AUTO_RESTART

    # Bring up the nodes
    self.roslaunch.prelaunch_check()
    self.roslaunch.load_parameters()
    self.roslaunch.launch_nodes()
    
    self.topic = rospy.TopicSub("/diagnostics", DiagnosticMessage, self.OnMsg)
    
    # Wait for our self-test service to come up
    #rospy.wait_for_service('urglaser/self_test', 5)

    # Try to query the self_test service on the hokuyo
    #try:
        #s = rospy.ServiceProxy('urglaser/self_test', SelfTest)
        #self.response = s.call(SelfTestRequest(),60)
    #except rospy.ServiceException, e:
      #wx.CallAfter(self.Cancel, "Could not contact node via ROS.\nError was: %s\nMake sure GUI is built correctly: rosmake qualification" % (e))
      #self.roslaunch.stop()
    #  return

    # If the self test passed, make the wall plot
    
    
  def OnMsg(self, msg):
    
    for i in range(len(msg.status)):
      if(msg.status[i].name=='MotorTest'):
          if(msg.status[i].level==0):
            self.Log(msg.status[i].message) 
            self.response.status.append(msg.status[i])
            if(self.count<self.testcount):
              mechanism.kill_controller('test_controller')
              self.OpenXml()
              self.Log("Starting Motor Test %s" % (count))
              mechanism.spawn_controller(self.xml)
              self.count=self.count+1
            else:
              self.finished=True
          else:
            self.Log(msg.status[i].message)
            self.response.status.append(msg.status[i])
            self.finished=True
      elif(msg.status[i].name=='EtherCAT Master' and self.ready==False):
        self.ready=True
        self.OpenXml()
        self.Log("Starting Motor Test 1")
        mechanism.spawn_controller(self.xml)
        self.count=self.count+1
    if self.finished:
      mechanism.shutdown()
      self.topic.unregister()
      self.roslaunch.stop()
      wx.CallAfter(self.Done, self.response)   
  
  def OpenXml(self):
    count = str(self.count)
    try:
      xmlFile =execution_path(str('xml/'+self.serial+'_test'+count+'.xml'))
      f = open(xmlFile)
      self.xml = f.read()
      f.close()
    except IOError:
      wx.CallAfter(self.Cancel, "Counld not open a test file: %s.\n" % (f))
      return 
  
      
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
    #self.topic = rospy.TopicSub("scan", LaserScan, self.OnLaserScan)



  def OnIdle(self, evt):
  
    evt.RequestMore(True)
