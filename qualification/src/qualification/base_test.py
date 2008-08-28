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
import os, sys

def execution_path(filename):
      return os.path.join(os.path.dirname(sys._getframe(1).f_code.co_filename), filename)

def NestPanel(top_panel, sub_panel):
  sizer = top_panel.GetSizer()
  if (sizer == None):
    sizer = wx.BoxSizer();
    top_panel.SetSizer(sizer)
  sizer.Clear(True)
  sizer.Add(sub_panel,1,wx.EXPAND)
  sizer.SetSizeHints(top_panel)
  top_panel.GetParent().Layout()

class BaseTest(object):
  def __init__(self, parent, panel, func):
    self.parent = parent
    self.panel = panel
    self.func = func
    self.prev = parent.GetSizer()
    for child in self.prev.GetChildren():
      child.GetWindow().Hide()
    parent.SetSizer(None,False)
    NestPanel(self.parent, self.panel)
  def Done(self, results):
    self.parent.GetSizer().Clear(True)
    self.parent.SetSizer(self.prev)
    for child in self.prev.GetChildren():
      child.GetWindow().Show()
    if self.func:
      self.func(results)
