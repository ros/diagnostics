#!/usr/bin/python
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Josh Faust


import os
import sys

WXVER = '2.8'
import wxversion
if wxversion.checkInstalled(WXVER):
  wxversion.select(WXVER)
else:
  print >> sys.stderr, "This application requires wxPython version %s"%(WXVER)
  sys.exit(1)

import wx

import roslib
roslib.load_manifest('qualification')

from optparse import OptionParser
import shutil
import glob
import traceback

import roslib.packages
import rospy

import rviz
import ogre_tools

from qualification.srv import *

def call_done_service(result, msg):
  result_service = rospy.ServiceProxy('visual_check', ScriptDone)
  r = ScriptDoneRequest()
  r.result = result
  r.failure_msg = msg
  rospy.wait_for_service('visual_check')
  result_service.call(r)

class VisualizerFrame(wx.Frame):
  def __init__(self, parent, id=wx.ID_ANY, title='Standalone Visualizer', pos=wx.DefaultPosition, size=(800, 600), style=wx.DEFAULT_FRAME_STYLE):
    wx.Frame.__init__(self, parent, id, title, pos, size, style)
    
    ogre_tools.initializeOgre()
    
    visualizer_panel = rviz.VisualizationPanel(self)
    
    self._visualizer_panel = visualizer_panel
    manager = visualizer_panel.getManager()
    
    self.Bind(wx.EVT_CLOSE, self.on_close)
    
    main_sizer = wx.BoxSizer(wx.VERTICAL)
    
    top_sizer = wx.BoxSizer(wx.VERTICAL)
    main_sizer.Add(top_sizer, 1, wx.EXPAND, 5)
    
    bottom_sizer = wx.BoxSizer(wx.HORIZONTAL)
    main_sizer.Add(bottom_sizer, 0, wx.ALIGN_RIGHT|wx.EXPAND, 5)
    
    top_sizer.Add(visualizer_panel, 1, wx.EXPAND, 5)
    
    self._instructions_ctrl = wx.TextCtrl(self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, wx.TE_MULTILINE|wx.TE_READONLY );
    self._pass_button = wx.Button(self, wx.ID_ANY, "Pass")
    self._fail_button = wx.Button(self, wx.ID_ANY, "Fail")
    bottom_sizer.Add(self._instructions_ctrl, 1, wx.ALL, 5)
    bottom_sizer.Add(self._pass_button, 0, wx.ALL|wx.ALIGN_BOTTOM, 5)
    bottom_sizer.Add(self._fail_button, 0, wx.ALL|wx.ALIGN_BOTTOM, 5)
    
    self.SetSizer(main_sizer)
    self.Layout()
    
    self._pass_button.Bind(wx.EVT_BUTTON, self.on_pass)
    self._fail_button.Bind(wx.EVT_BUTTON, self.on_fail)
    
          
  def on_close(self, event):
    self.Destroy()
    
  def on_pass(self, event):
    call_done_service(ScriptDoneRequest.RESULT_OK, 'Visual check passed by operator.')
    self.Destroy()

  def on_fail(self, event):
    call_done_service(ScriptDoneRequest.RESULT_FAIL, 'Visual check failed by operator.')
    self.Destroy()
      
  def load_config_from_path(self, path):
    manager = self._visualizer_panel.getManager()
    manager.removeAllDisplays()
    config = wx.FileConfig(localFilename=path)
    manager.loadGeneralConfig(config)
    manager.loadDisplayConfig(config)
    
  def set_instructions(self, instructions):
    self._instructions_ctrl.SetValue(instructions)
      
  def on_open(self, event):
    dialog = wx.FileDialog(self, "Choose a file to open", self._save_location, wildcard="*."+self._CONFIG_EXTENSION, style=wx.FD_OPEN)
    if dialog.ShowModal() == wx.ID_OK:
      path = dialog.GetPath()
      self.load_config_from_path(path)

class VisualizerApp(wx.App):
  def __init__(self, file):
    self._filepath = file
    self._instructions = 'Move joints and verify robot is OK.'
    
    wx.App.__init__(self)
  
  def OnInit(self):
    try:
      frame = VisualizerFrame(None, wx.ID_ANY, "Visual Verifier", wx.DefaultPosition, wx.Size( 800, 600 ) )
    
      if (not os.path.exists(self._filepath)):
        call_done_service(ScriptDoneRequest.RESULT_ERROR, 'Visual check recorded error, file does not exist!')
        rospy.logerr('Visual check recorded error, file does not exist!')
        #print "File '%s' does not exist!"%(self._filepath)
        #sys.exit(1)
    
      frame.load_config_from_path(self._filepath)
      frame.set_instructions(self._instructions)
      frame.Show(True)
      return True
    except:
      # Fail here, because this could mean TF is giving NaN values (encoder failure...)
      call_done_service(ScriptDoneRequest.RESULT_ERROR, 'Visual check recorded error on initialization: %s' % traceback.format_exc())
      rospy.logerr('Error initializing rviz: %s' % traceback.format_exc())

  def OnExit(self):
    ogre_tools.cleanupOgre()

if __name__ == "__main__":
  if (len(sys.argv) < 2):
    call_done_service(ScriptDoneRequest.RESULT_ERROR, "Visual verifier not given path to config file.\nusage: visual_verifier.py 'path to config file'")
    #print "usage: visual_verifier.py 'path to config file'"
    #sys.exit(1)
  
  try:
    app = VisualizerApp(sys.argv[1])
    app.MainLoop()
  except:
    call_done_service(ScriptDoneRequest.RESULT_ERROR, 'Visual check recorded error: %s' % traceback.format_exc())
    rospy.logerr(traceback.format_exc())
