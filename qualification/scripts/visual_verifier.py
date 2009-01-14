#!/usr/bin/python

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

import rostools
import rostools.packspec
rostools.update_path('qualification')
import rospy

from optparse import OptionParser

import shutil
import glob
import ogre_visualizer
import ogre_tools

from qualification.srv import *

class VisualizerFrame(wx.Frame):
  def __init__(self, parent, id=wx.ID_ANY, title='Standalone Visualizer', pos=wx.DefaultPosition, size=(800, 600), style=wx.DEFAULT_FRAME_STYLE):
    wx.Frame.__init__(self, parent, id, title, pos, size, style)
    
    ogre_tools.initializeOgre()
    
    visualizer_panel = ogre_visualizer.VisualizationPanel(self)
    
    self._package_path = rostools.packspec.get_pkg_dir('ogre_visualizer')
    self._global_config_path = os.path.join(self._package_path, "configs")
    
    self._visualizer_panel = visualizer_panel
    
    media_path = rostools.packspec.get_pkg_dir( "gazebo_robot_description" )
    media_path += "/world/Media/";
    
    media_paths = [media_path]
    media_paths.append( media_path )
    media_paths.append( media_path + "fonts" )
    media_paths.append( media_path + "materials" )
    media_paths.append( media_path + "materials/scripts" )
    media_paths.append( media_path + "materials/programs" )
    media_paths.append( media_path + "materials/textures" )
    media_paths.append( media_path + "models" )
    media_paths.append( media_path + "models/pr2" )
    
    ogre_tools.initializeResources( media_paths )
    
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
    result_service = rospy.ServiceProxy('test_result', TestResult)
    r = TestResultRequest()
    r.plots = []
    r.text_result = "Visual Verification Succeeded"
    r.result = TestResultRequest.RESULT_PASS
    rospy.wait_for_service('test_result')
    result_service.call(r)
    
    
  def on_fail(self, event):
    result_service = rospy.ServiceProxy('test_result', TestResult)
    r = TestResultRequest()
    r.plots = []
    r.text_result = "Visual Verification Failed"
    r.result = TestResultRequest.RESULT_FAIL
    rospy.wait_for_service('test_result')
    result_service.call(r)
      
  def load_config_from_path(self, path):
    manager = self._visualizer_panel.getManager()
    manager.removeAllDisplays()
    config = wx.FileConfig(localFilename=path)
    manager.loadConfig(config)
    
  def set_instructions(self, instructions):
    self._instructions_ctrl.SetValue(instructions)
      
  def on_open(self, event):
    dialog = wx.FileDialog(self, "Choose a file to open", self._save_location, wildcard="*."+self._CONFIG_EXTENSION, style=wx.FD_OPEN)
    if dialog.ShowModal() == wx.ID_OK:
      path = dialog.GetPath()
      self.load_config_from_path(path)

class VisualizerApp(wx.App):
  def __init__(self, file, instructions):
    self._filepath = file
    self._instructions = instructions
    
    wx.App.__init__(self)
  
  def OnInit(self):
    frame = VisualizerFrame(None, wx.ID_ANY, "Visual Verifier", wx.DefaultPosition, wx.Size( 800, 600 ) )
    
    if (not os.path.exists(self._filepath)):
      print "File '%s' does not exist!"%(self._filepath)
      sys.exit(1)
    
    frame.load_config_from_path(self._filepath)
    frame.set_instructions(self._instructions)
    frame.Show(True)
    return True
        
  def OnExit(self):        
    ogre_tools.cleanupOgre()

if __name__ == "__main__":
  if (len(sys.argv) < 3):
    print "usage: visual_verifier.py 'path to config file' 'instruction text'"
    sys.exit(1)
  
  app = VisualizerApp(sys.argv[1], sys.argv[2])
  app.MainLoop()
