#! /usr/bin/env python

import rostools
rostools.update_path('qualification_gui')

import rospy, sys
import os, signal, popen2
import time
import wx

import random
import unittest

import wx.lib.plot as plot

from wx import xrc

class QualificationApp(wx.App):
  def OnInit(self):
    self.res = xrc.XmlResource('gui.xrc')

    self.frame = self.res.LoadFrame(None, 'qualificationFrame');
    self.panel = xrc.XRCCTRL(self.frame, 'qualificationPanel')

    self.testPanel = self.res.LoadPanel(self.panel, 'scanPanel')

    self.sizer = wx.BoxSizer(wx.VERTICAL)
    
    self.sizer.Add(self.testPanel,1,wx.EXPAND)
    self.sizer.SetSizeHints(self.panel)
    self.panel.SetSizer(self.sizer)

    self.frame.Bind(wx.EVT_BUTTON, self.OnTest, id=xrc.XRCID('testButton'))
    self.frame.Bind(wx.EVT_BUTTON, self.OnStop, id=xrc.XRCID('stopButton'))

    sizer2 = self.frame.GetSizer()
    sizer2.SetSizeHints(self.frame)
    self.frame.SetSize(wx.Size(800,800))
    self.frame.Layout()
    
    self.frame.Show(True)
    self.frame.Centre()
    return True

  def OnTest(self, evt):
    self.sizer.Clear(True)

    self.plotPanel = plot.PlotCanvas(self.panel,-1)
    self.sizer.Add(self.plotPanel,1,wx.EXPAND)
    markers = plot.PolyLine([(0,0)])
    gc = plot.PlotGraphics([(markers)], 'Motor Data', 'Time', 'Velocity')
    self.plotPanel.Draw(gc)
    
    self.testPanel = self.res.LoadPanel(self.panel, 'testPanel')
    self.sizer.Add(self.testPanel,0,wx.EXPAND)

    self.sizer.SetSizeHints(self.panel)
    self.panel.SetSizer(self.sizer)
    self.frame.Layout()

  def OnStop(self, evt):
    self.sizer.Clear(True)
    self.testPanel = self.res.LoadPanel(self.panel, 'scanPanel')
    self.sizer.Add(self.testPanel,1,wx.EXPAND)
    self.sizer.SetSizeHints(self.panel)
    self.panel.SetSizer(self.sizer)
    self.frame.Layout()


if __name__ == '__main__':
  try:
    app = QualificationApp(0)
    app.MainLoop()
  except Exception, e:
    print e
    
  print 'quit'
  rospy.signal_shutdown('GUI shutdown')
