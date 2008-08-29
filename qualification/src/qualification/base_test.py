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
import os, sys, time, datetime

from wx import xrc

def execution_path(filename):
      return os.path.join(os.path.dirname(sys._getframe(1).f_code.co_filename), filename)

def NestPanel(top_panel, sub_panel):

  start_size = top_panel.GetTopLevelParent().GetSize()

  sizer = top_panel.GetSizer()
  if (sizer == None):
    sizer = wx.BoxSizer();
    top_panel.SetSizer(sizer)
  sizer.Clear(True)
  sizer.Add(sub_panel,1,wx.EXPAND)
  sizer.SetSizeHints(top_panel)

  sizer2 = top_panel.GetTopLevelParent().GetSizer()
  sizer2.SetSizeHints(top_panel.GetTopLevelParent())

  top_panel.GetTopLevelParent().SetSize(start_size)
  top_panel.GetTopLevelParent().Layout()


class BaseTest(object):
  def __init__(self, parent, instruct_panel, test_panel, func):
    self.parent = parent
    self.instruct_panel = instruct_panel
    self.test_panel = test_panel
    self.func = func

    #Save previous panel state and hide it
    self.prev = parent.GetSizer()
    for child in self.prev.GetChildren():
      child.GetWindow().Disable()
      child.GetWindow().Hide()

    #Set sizer to None non-destructively to avoid blowing away with NestPanel
    parent.SetSizer(None,False)

    #Load instruction wrapper panel
    self.res = xrc.XmlResource(execution_path('base_test.xrc'))
    self.instruct_wrapper = self.res.LoadPanel(self.parent, 'instruct_wrapper')
    self.instruct_wrapper_panel = xrc.XRCCTRL(self.instruct_wrapper, 'instruct_wrapper_panel')

    self.instruct_wrapper.Bind(wx.EVT_BUTTON, self.OnContinue, id=xrc.XRCID('continue_button'))
    self.instruct_wrapper.Bind(wx.EVT_BUTTON, self.OnCancel, id=xrc.XRCID('cancel_button'))

    self.instruct_panel.Reparent(self.instruct_wrapper_panel)

    NestPanel(self.instruct_wrapper_panel, self.instruct_panel)
    NestPanel(self.parent, self.instruct_wrapper)


  def OnContinue(self, evt):
    NestPanel(self.parent, self.test_panel)
    self.Run()

  def OnCancel(self, evt):
    self.Done(None)

  def Run(self):
    print 'It appears Run was not overridden'
    self.Done(None)

  def Log(self, msg):
    try:
      if self.parent.GetTopLevelParent().log:
        self.parent.GetTopLevelParent().log.AppendText(datetime.datetime.now().strftime("%Y-%m-%d_%I:%M:%S: ") + msg + '\n')
    except AttributeError:
      pass

  def Done(self, results):
    if self.func:
      self.func(results)

    start_size = self.parent.GetTopLevelParent().GetSize()

    self.parent.GetSizer().Clear(True)
    self.parent.SetSizer(self.prev)

    for child in self.prev.GetChildren():
      child.GetWindow().Enable()
      child.GetWindow().Show()

    sizer = self.parent.GetSizer()
    sizer.SetSizeHints(self.parent)

    sizer2 = self.parent.GetTopLevelParent().GetSizer()
    sizer2.SetSizeHints(self.parent.GetTopLevelParent())
    self.parent.GetTopLevelParent().Layout()
    self.parent.SetFocus()

    self.parent.GetTopLevelParent().SetSize(start_size)
    self.parent.GetTopLevelParent().Layout()
