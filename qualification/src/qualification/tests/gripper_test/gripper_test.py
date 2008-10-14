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
import thread

import rospy
import roslaunch

from qualification import *

from robot_msgs.msg import *
from std_msgs.msg import *
from robot_srvs.srv import *
from mechanism_control.msg import *
from mechanism_control import mechanism

import time

class GripperTest(BaseTest):
  def __init__(self, parent, serial, func):
    self.ready=False
    self.serial = serial[0:7]
    self.count = 1
    self.finished=False
    self.testcount=1
    # Load the XRC resource
    self.res2 = xrc.XmlResource(execution_path('gripper_test.xrc'))
    

    # Initialize the BaseTest with these parts
    BaseTest.__init__(self, parent, serial, func, 'gripper Test')

    self.roslaunch = None
    self.topic = None
    self.data_topic= None
    self.response = DiagnosticMessage(None,[])
    self.data_dict = {}
  
  def instruct_panel_gen(self, parent):
    # Load the instruction and test panels from the XRC resource
    panel = 'instruct_panel_'+self.serial
    return self.res2.LoadPanel(parent, panel )
  
  def test_panel_gen(self, parent):
    self.test_panel =self.res2.LoadPanel(parent, 'test_panel')
    return self.res2.LoadPanel(parent, 'test_panel')
  
  # This is what runs once the instructions are read
  def Run(self):
    thread.start_new_thread(self.RunThread, (None,))

  def RunThread(self,arg):
    #Create a roslauncher
    self.roslaunch = roslaunch.ROSLauncher()
    loader = roslaunch.XmlLoader()

    # Try loading the test XML file
    try:
        xmlFile =execution_path(str('xml/'+self.serial+'_gripper_test.xml'))
        loader.load(xmlFile, self.roslaunch)
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
    self.data_topic = rospy.TopicSub("/gripper_test_data", ChannelFloat32, self.OnData)
    self.test_panel.Bind(wx.EVT_BUTTON, self.EStop, id=xrc.XRCID('ESTOP_button'))
    
  def OnMsg(self, msg):
    for i in range(len(msg.status)):
      if(msg.status[i].name=='GripperTest'):
          if(msg.status[i].level==0):
            self.Log(msg.status[i].message) 
            self.response.status.append(msg.status[i])
            if(self.count<self.testcount+1):
              mechanism.kill_controller('test_controller')
              self.OpenXml()
              self.Log("Starting gripper Test %s" % (self.count))
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
        self.Log("Starting gripper Test 1")
        mechanism.spawn_controller(self.xml)
        self.count=self.count+1
    if self.finished:
      time.sleep(2)
      for v in self.data_dict:
        print v
      mechanism.shutdown()
      self.topic.unregister()
      self.data_topic.unregister()
      self.roslaunch.stop()
      wx.CallAfter(self.FinishTest)
  
  def OnData(self,msg):
    print 'Got data named %s' % (msg.name)
    self.data_dict[msg.name] = msg.vals
  
  def VisDone(self,evt):
    wx.CallAfter(self.Done, self.response, self.data_dict)
  
  def FinishTest(self):
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
    vis_panel.Bind(wx.EVT_BUTTON, self.VisDone, id=xrc.XRCID('done_button'))

    # Plot the values and line of best fit
    fig=self.plot.get_figure()
    axes1 = fig.add_subplot(211)
    axes1.clear()
    axes2 = fig.add_subplot(212)
    axes2.clear()
    axes1.plot(numpy.array(self.data_dict['position']), numpy.array(self.data_dict['effort']), 'r--')
    axes2.plot(numpy.array(self.data_dict['position']), numpy.array(self.data_dict['velocity']), 'b--')
    
    # Adjust the size of the axes using size of the window (this is somewhat hackish)
    #sz = self.plot.GetSize()
    #xrng = 1.0
    #yrng = 2.0*float(sz[1])/sz[0] * xrng;
    #axes.axis([-xrng,xrng,0,yrng])
    #axes.grid()
    
    self.plot.draw()
    self.plot.Show()
  
  
  #open the controller xml file
  def OpenXml(self):
    count = str(self.count)
    try:
      xmlFile =execution_path(str('xml/'+self.serial+'_test'+count+'.xml'))
      f = open(xmlFile)
      self.xml = f.read()
      f.close()
    except IOError:
      wx.CallAfter(self.Cancel, "Counld not open a test file: %s.\n" % (f))
      self.finished=True
      return 
  
  def EStop(self, evt):
    self.response.status.append(DiagnosticStatus(2, 'ESTOP',"The ESTOP button was pressed.",[],[]))
    self.Log("Emergency Stop: Ending Test.")
    mechanism.shutdown()
    self.topic.unregister()
    self.data_topic.unregister()
    self.roslaunch.stop()
    wx.CallAfter(self.Done, self.response)
      

  def OnIdle(self, evt):
  
    evt.RequestMore(True)
