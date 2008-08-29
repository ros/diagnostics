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

class HokuyoTest(BaseTest):
  def __init__(self, parent, func):

    res = xrc.XmlResource(execution_path('hokuyo_test.xrc'))

    instruct_panel = res.LoadPanel(parent, 'instruct_panel')
    test_panel = res.LoadPanel(parent, 'hokuyo_test')

    BaseTest.__init__(self, parent, instruct_panel, test_panel, func)

  # This is what runs once the instructions are read
  def Run(self):
    worker = HokuyoThread(self)

class HokuyoThread(Thread):
  def __init__(self, test):
    Thread.__init__(self)
    self.test = test
    self.start()

  def run(self):

    # Create a roslauncher
    rl = roslaunch.ROSLauncher()
    loader = roslaunch.XmlLoader()

    # Try loading the XML file
    try:
        loader.load(execution_path('hokuyo_test.xml'), rl)
    except roslaunch.XmlParseException, e:
      wx.CallAfter(self.test.Done, 'Could not load XML file')
      return

    # Make sure we get a fresh master
    rl.master.auto = rl.master.AUTO_RESTART

    # Bring up the nodes
    rl.prelaunch_check()
    rl.load_parameters()
    rl.launch_nodes()

    # Wait for our self-test service to come up
    rospy.wait_for_service('urglaser/self_test')

    # Try to query the self_test service on the hokuyo
    try:
        s = rospy.ServiceProxy('urglaser/self_test', SelfTest)
        resp = s()
    except rospy.ServiceException, e:
      wx.CallAfter(self.test.Done, 'Could not contact node via ROS')
      return

    # Bring down the nodes
    rl.stop()

    # When the thread is done, raise our response
    wx.CallAfter(self.test.Done, resp)

