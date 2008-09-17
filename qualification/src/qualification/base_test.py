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

import os, sys, time, datetime

import wx
from wx import xrc

def execution_path(filename):
      return os.path.join(os.path.dirname(sys._getframe(1).f_code.co_filename), filename)

def NestPanel(top_panel, sub_panel):

  # Cache the original size of the top-level frame
  start_size = top_panel.GetTopLevelParent().GetSize()

  # Get the top_level's sizer
  sizer = top_panel.GetSizer()

  # Create a new one if it doesn't exist
  if (sizer == None):
    sizer = wx.BoxSizer();
    top_panel.SetSizer(sizer)

  # Clear the sizer (destructively)
  sizer.Clear(True)

  # Add our sub_panel to the sizer
  sizer.Add(sub_panel,1,wx.EXPAND)

  # Reset size hints for the top_panel
  sizer.SetSizeHints(top_panel)

  # Also reset size hints for the top level frame
  sizer2 = top_panel.GetTopLevelParent().GetSizer()
  sizer2.SetSizeHints(top_panel.GetTopLevelParent())

  # Reset size to our cached size, and then layout
  top_panel.GetTopLevelParent().SetSize(start_size)
  top_panel.GetTopLevelParent().Layout()

class BaseTest(object):
  def __init__(self, parent, serial, func, desc):
    # Store variables
    self.parent = parent
    self.serial = serial
    self.func = func
    self.desc = desc

    self.date = datetime.datetime.now()

    # Save previous panel state and hide it
    self.prev = parent.GetSizer()
    for child in self.prev.GetChildren():
      child.GetWindow().Disable()
      child.GetWindow().Hide()

    # Set sizer to None non-destructively to avoid blowing away with NestPanel in the future
    parent.SetSizer(None,False)

    self.DoInstruct()

  def DoInstruct(self):
    # Load panels
    self.res = xrc.XmlResource(execution_path('base_test.xrc'))
    self.instruct_wrapper = self.res.LoadPanel(self.parent, 'instruct_wrapper')
    self.instruct_wrapper_panel = xrc.XRCCTRL(self.instruct_wrapper, 'instruct_wrapper_panel')

    # Bind the continue/cancel buttons
    self.instruct_wrapper.Bind(wx.EVT_BUTTON, self.OnContinue, id=xrc.XRCID('ins_continue_button'))
    self.instruct_wrapper.Bind(wx.EVT_BUTTON, self.OnCancel, id=xrc.XRCID('ins_cancel_button'))

    # Create the instruction panel using the child classes generator
    self.instruct_panel = self.instruct_panel_gen(self.instruct_wrapper_panel)

    # Place the instruction panel inside the wrapper
    NestPanel(self.instruct_wrapper_panel, self.instruct_panel)

    # Place the wrapper inside the parent
    NestPanel(self.parent, self.instruct_wrapper)
        

  def OnContinue(self, evt):
    self.test_panel     = self.test_panel_gen(self.parent)
    # Destructively replace the contents of parent with the test panel
    NestPanel(self.parent, self.test_panel)
    # Run the test
    self.Run()

  def OnCancel(self, evt):
    self.Cancel('Self test was canceled')

  def Run(self):
    self.Cancel('The invoked test had no Run defined')

  def Log(self, msg):
    try:
      # I'm sure a wx python logging infrastructure exists... I just can't figure it out
      if self.parent.GetTopLevelParent().log:
        self.parent.GetTopLevelParent().log.AppendText(datetime.datetime.now().strftime("%Y-%m-%d_%I:%M:%S: ") + msg + '\n')
    except AttributeError:
      pass

  def Cancel(self, msg):
    self.Finish(msg)

  def Finish(self, msg):
    # Invoke the callback with the results, if defined
    if self.func:
      self.func(msg)

    # Cache the original size
    start_size = self.parent.GetTopLevelParent().GetSize()

    # Destroy contents of the old sizer
    self.parent.GetSizer().Clear(True)
    
    # Restore the original sizer:
    self.parent.SetSizer(self.prev)

    # Re-enable all the original children
    for child in self.prev.GetChildren():
      child.GetWindow().Enable()
      child.GetWindow().Show()

    # Restore size-hint stuff for parent
    sizer = self.parent.GetSizer()
    sizer.SetSizeHints(self.parent)

    # Restore size-hint stuff for top level (This seems gratuitous but is necessary)
    sizer2 = self.parent.GetTopLevelParent().GetSizer()
    sizer2.SetSizeHints(self.parent.GetTopLevelParent())

    # Resize to the original size (since sizehints otherwise thwarts this)
    self.parent.GetTopLevelParent().SetSize(start_size)
    self.parent.GetTopLevelParent().Layout()

    # Give parent focus
    self.parent.SetFocus()    

  def Done(self, results, data=None):
    self.data = data

    self.results_panel = self.res.LoadPanel(self.parent, 'results_panel')
    self.result_ctrl = xrc.XRCCTRL(self.results_panel, 'result_ctrl')

    self.results_panel.Bind(wx.EVT_BUTTON, self.OnSubmit, id=xrc.XRCID('res_submit_button'))
    self.results_panel.Bind(wx.EVT_BUTTON, self.OnDiscard, id=xrc.XRCID('res_discard_button'))

    NestPanel(self.parent, self.results_panel)

    statdict = {0: 'OK', 1: 'WARN', 2: 'ERROR'}

    passfail = 'FAIL'

    res = ''

    # If the result has an attribute called 'status', it is probably an array of DiagnosticStatus's
    if (hasattr(results, 'status')):
      passfail = 'PASS'
      i = 1
      for stat in results.status:
        if (stat.level > 1):
          passfail = 'FAIL'
        res += 'Test %2d) %s\n' % (i, stat.name)
        res += '  [%s]: %s\n' % (statdict[stat.level], stat.message)
        for val in stat.values:
          res += '   [%s] = %f\n' % (val.label, val.value)
        i += 1

    head = '----------------------------------------\n'
    head += 'Serial: %s\n' % (self.serial)
    head += 'Desc: %s\n' % (self.desc)
    head += 'Date: %s\n' % (self.date.strftime("%m/%d/%Y %I:%M:%S"))
    head += 'Result: %s\n' % (passfail)
    head += '----------------------------------------\n'


    self.result_ctrl.SetValue(head + res)

  def OnDiscard(self, evt):
    self.DoInstruct()

  def OnSubmit(self, evt):
    ###
    ### ADD CODE HERE TO actually submit log!
    ###
    fname = '%s_%s.test' % (self.serial, self.date.strftime("%Y_%m_%d_%I_%M_%S"))
    f = open(fname, 'w')
    f.write(self.result_ctrl.GetValue())
    
    if (self.data != None):
      f.write('----------------------------------------\n')
      for d in self.data:
        f.write('%s = ' % (d))
        for v in self.data[d]:
          f.write('%f, ' % (v))
        f.write('\n')

    f.close()
        
    self.Finish('Self test completed')
