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
from robot_srvs.srv import *
from mechanism_control.msg import *
from mechanism_control import mechanism


class MotorTest(BaseTest):
  def __init__(self, parent, serial, func):
    self.ready=False
    self.serial1 = serial[0:7]
    self.count = 1
    self.finished=False
    self.testcount=4
    # Load the XRC resource
    self.res2 = xrc.XmlResource(execution_path('motor_test.xrc'))
    

    # Initialize the BaseTest with these parts
    BaseTest.__init__(self, parent, serial, func, 'Motor Test')

    self.roslaunch = None
    self.topic = None
    self.response = DiagnosticMessage(None,[])
  
  def instruct_panel_gen(self, parent):
    # Load the instruction and test panels from the XRC resource
    panel = 'instruct_panel_'+self.serial1
    return self.res2.LoadPanel(parent, panel )
  
  def test_panel_gen(self, parent):
    self.test_panel =self.res2.LoadPanel(parent, 'test_panel')
    return self.res2.LoadPanel(parent, 'test_panel')
  
  # This is what runs once the instructions are read
  def Run(self):
    thread.start_new_thread(self.RunThread, (None,))

  def RunThread(self,arg):
    #Create a roslauncher
    config = roslaunch.ROSLaunchConfig()
    self.count = 1
    self.finished=False
    self.testcount=4
    self.ready=False

    # Try loading the test XML file
    try:
        xmlFile = execution_path(str('xml/'+self.serial1+'/'+self.serial1+'_motor_test.xml'))
        loader = roslaunch.XmlLoader()
        loader.load(xmlFile, config)
    except roslaunch.XmlParseException, e:
        wx.CallAfter(self.Cancel, 'Could not load back-end XML to launch necessary nodes.  Make sure the GUI is up to date.')
        return

    # Make sure we get a fresh master
    config.master.auto = config.master.AUTO_RESTART

    # Bring up the nodes
    self.roslaunch = roslaunch.ROSLaunchRunner()
    self.roslaunch.launch()
    
    self.topic = rospy.TopicSub("/diagnostics", DiagnosticMessage, self.OnMsg)
    self.test_panel.Bind(wx.EVT_BUTTON, self.EStop, id=xrc.XRCID('ESTOP_button'))
    
  def OnMsg(self, msg):
    for i in range(len(msg.status)):
      if(msg.status[i].name=='MotorTest'):
          if(msg.status[i].level==0):
            self.Log(msg.status[i].message) 
            self.response.status.append(msg.status[i])
            if(self.count<self.testcount+1):
              mechanism.kill_controller('test_controller')
              self.OpenXml()
              self.Log("Starting Motor Test %s" % (self.count))
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
  
  #open the controller xml file
  def OpenXml(self):
    count = str(self.count)
    try:
      xmlFile =execution_path(str('xml/'+self.serial1+'/'+self.serial1+'_test'+count+'.xml'))
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
    self.roslaunch.stop()
    wx.CallAfter(self.Done, self.response)
      

  def OnIdle(self, evt):
  
    evt.RequestMore(True)
