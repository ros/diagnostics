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

import time

from qualification import *
from robot_msgs.msg import *
from threading import *

class HokuyoThread(Thread):
  def __init__(self, test):
    Thread.__init__(self)
    self.test = test
    self.start()

  def run(self):
    for i in range(10):
      time.sleep(1)

    out_stat = []
    out_stat.append(DiagnosticStatus(0, 'Stat 1', 'Something passed', []))
    out_stat.append(DiagnosticStatus(1, 'Stat 2', 'Something else warned', [DiagnosticValue(42.1, 'foo')]))
    out = DiagnosticMessage(None, out_stat)
    wx.CallAfter(self.test.Done, out)


class HokuyoTest(BaseTest):
  def __init__(self, parent, func):
    self.parent = parent
    self.res = xrc.XmlResource(execution_path('hokuyo_test.xrc'))
    self.panel = self.res.LoadPanel(self.parent, 'hokuyo_test')
    BaseTest.__init__(self, parent, self.panel, func)

    worker = HokuyoThread(self)
