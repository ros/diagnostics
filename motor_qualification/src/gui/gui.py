#! /usr/bin/env python
# Provides quick access to the services exposed by MechanismControlNode

import rostools
rostools.update_path('mechanism_control')
rostools.update_path('std_srvs')

from mechanism_control.srv import *
from mechanism_control.msg import *


import rospy, sys
import os, signal, popen2
import time
import wx
import wx.lib.plot as plot
import motor_tests as test
import roscommands
import error_codes as error

class RosFrame(wx.Frame):
  def __init__(self, parent, id):
    wx.Frame.__init__(self, parent, id, 'Motor Test', size=(800,800))

    test.test_topic(self)
    
    self.testing = 0 
    self.output=0
    self.data =[(0,0)]
    self.data2 =[(0,0)]
    
    panel = wx.Panel(self, -1)
    sizer = wx.GridBagSizer(4, 4) 
    text1 = wx.StaticText(panel, -1, test.plot_title)
    sizer.Add(text1, (0, 0), flag=wx.TOP | wx.LEFT | wx.BOTTOM, border=5)



    self.test_panel = plot.PlotCanvas(panel,-1)
    test.drawPanel(self)
    sizer.Add(self.test_panel, (1, 0), (5, 3), wx.EXPAND | wx.LEFT | wx.RIGHT, 5)                

    text2 = wx.StaticText(panel, -1, 'Test Status')
    sizer.Add(text2, (6, 0), flag=wx.TOP | wx.LEFT | wx.BOTTOM, border=5)
    
    self.text = wx.TextCtrl(panel, -1, style=wx.TE_MULTILINE)
    sizer.Add(self.text, (7, 0), (3, 3), wx.EXPAND | wx.LEFT | wx.RIGHT, 5)


    buttonStart = wx.Button(panel, 106, "Start", size=(90, 28))
    buttonStop = wx.Button(panel, 107, "Stop", size=(90, 28))
    sizer.Add(buttonStart, (10, 1))
    sizer.Add(buttonStop, (10, 2), flag=wx.RIGHT | wx.BOTTOM, border=5)

    sizer.AddGrowableCol(0)
    sizer.AddGrowableRow(3)
    sizer.AddGrowableRow(5)
    sizer.SetEmptyCellSize((5, 5))
    panel.SetSizer(sizer)

    self.Bind(wx.EVT_BUTTON, self.start, id=106)
    self.Bind(wx.EVT_BUTTON, self.stop, id=107)
    self.Bind(wx.EVT_IDLE, self.onIdle)
    self.Centre()
  
     
  def onIdle(self, event):
    event.RequestMore(True)
    test.drawPanel(self)
    #self.drawPlot()
    
  def start(self, event):
    if self.testing == 0:
      dlg = wx.TextEntryDialog(self, 'Scan the part','Part SN:')
      if dlg.ShowModal() == wx.ID_OK:
        self.text.SetDefaultStyle(wx.TextAttr(wx.BLACK))
        test.scan_check(self,dlg.GetValue())
    else:
      dlg = wx.MessageDialog(self, 'Test already in Progress','Error', wx.OK|wx.ICON_ERROR)
      dlg.ShowModal()
    dlg.Destroy()

  def stop(self, event):
    if(self.testing):
      roscommands.end_test()
      dlg = wx.MessageDialog(self, 'Ending Test','User Hault', wx.OK|wx.ICON_HAND)
      self.testing=0
      dlg.ShowModal()
    else:
      error.error(self,'NO_TEST')
    return 
   
  def testError(self):
    self.file.write("TEST FAILED\n")
    roscommands.end_test()
    dlg = wx.MessageDialog(self, 'TEST ERROR:Ending Test','ERROR', wx.OK|wx.ICON_EXCLAMATION)
    self.testing=0
    dlg.ShowModal()
    
  def testDone(self):
    self.file.write("TEST COMPLETE\n")
    roscommands.end_test()
    dlg = wx.MessageDialog(self, 'TEST COMPLETE:Ending Test','Done', wx.OK)
    self.testing=0
    dlg.ShowModal()

  def testPassed (self, name):
    self.text.SetDefaultStyle(wx.TextAttr(wx.BLACK))
    self.text.AppendText(name + ":")
    self.text.SetDefaultStyle(wx.TextAttr(wx.GREEN))
    self.text.AppendText(" passed\n")
    
  def testFailed (self, name):
    self.text.SetDefaultStyle(wx.TextAttr(wx.BLACK))
    self.text.AppendText(name +":")
    self.text.SetDefaultStyle(wx.TextAttr(wx.RED))
    self.text.AppendText(" failed\n")
      
class RosApp(wx.App):
  def OnInit(self):
    frame = RosFrame(None, -1)
    
    frame.Show(True)
    frame.Centre()
    return True


if __name__ == '__main__':
  sub = popen2.Popen3('botherder')
  time.sleep(2)
  
  try:
    app = RosApp(0)
    app.MainLoop()
  except Exception, e:
    print e
    
  os.killpg(os.getpgid(sub.pid), signal.SIGHUP)
    
  print 'quit'
  rospy.signal_shutdown('GUI shutdown')


    
