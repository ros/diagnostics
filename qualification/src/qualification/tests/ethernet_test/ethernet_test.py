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


from threading import Thread

from qualification import *

from robot_msgs.msg import *
from robot_srvs.srv import *
import rospy
import roslaunch

import thread

class EthernetTest(BaseTest):
  def __init__(self, parent, serial, func):

    # Load the XRC resource
    self.res2 = xrc.XmlResource(execution_path('ethernet_test.xrc'))

    # Initialize the BaseTest with these parts
    BaseTest.__init__(self, parent, serial, func, 'Ethernet Cable')

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
      loader.load(execution_path('ethernet_test.xml'), config)
    except roslaunch.XmlParseException, e:
      wx.CallAfter(self.Cancel, 'Could not load back-end XML to launch necessary nodes.  Make sure the GUI is up to date.')
      return

    # Make sure we get a fresh master
    config.master.auto = config.master.AUTO_RESTART

    # Bring up the nodes
    rl = roslaunch.ROSLaunchRunner(config)
    rl.launch()

    # Wait for our self-test service to come up
    rospy.wait_for_service('ethernet/self_test', 20)
    
    # Try to query the self_test service on the imu
    try:
        s = rospy.ServiceProxy('ethernet/self_test', SelfTest)
        resp = s.call(SelfTestRequest(),120)
    except rospy.ServiceException, e:
      print e
      wx.CallAfter(self.Cancel, "Could not contact node via ROS.\nError was: %s\nMake sure GUI is built correctly: rosmake qualification" % (e))
      rl.stop()
      return

    # Bring down the nodes
    rl.stop()

    # When the thread is done, raise our response
    wx.CallAfter(self.Done, resp)
