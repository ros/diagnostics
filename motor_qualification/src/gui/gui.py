#! /usr/bin/env python
# Provides quick access to the services exposed by MechanismControlNode

import rostools
rostools.update_path('mechanism_control')
rostools.update_path('generic_controllers')

from mechanism_control.srv import *
from generic_controllers.srv import *
import rostools.packspec as packspec
import rospy, sys
import os, signal, popen2
import time
import wx
import wx.lib.plot as plot


class RosFrame(wx.Frame):
  def __init__(self, parent, id):
    wx.Frame.__init__(self, parent, id, 'Motor Test', size=(800,800))
   
    panel = wx.Panel(self, -1)
    sizer = wx.GridBagSizer(4, 4)

    self.testing = 0 
    self.data = [(1,2), (2,3), (3,5), (4,6), (5,8), (6,8), (10,10)]

    text1 = wx.StaticText(panel, -1, 'Motor Response')
    sizer.Add(text1, (0, 0), flag=wx.TOP | wx.LEFT | wx.BOTTOM, border=5)


    client = plot.PlotCanvas(panel,-1)
    markers = plot.PolyMarker(self.data, legend='', colour='blue', marker='circle', size=1)
    gc = plot.PlotGraphics([markers], 'Motor Data', 'Time', 'Velocity')
    client.Draw(gc, xAxis=(0,15), yAxis=(0,15))
    sizer.Add(client, (1, 0), (5, 3), wx.EXPAND | wx.LEFT | wx.RIGHT, 5)                

    text2 = wx.StaticText(panel, -1, 'Status')
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
    self.Centre()
     
  def start(self, event):
    if self.testing == 0:
      dlg = wx.TextEntryDialog(self, 'Scan the motor','Motor Model')
      if dlg.ShowModal() == wx.ID_OK:
        if self.check_motor(dlg.GetValue()):
          self.text.SetDefaultStyle(wx.TextAttr(wx.BLACK))
          self.text.AppendText('Starting Motor Test for : %s \n' % dlg.GetValue())
          self.testing = 1
          start_test(dlg.GetValue(),self)          
        else:
          self.error('NOT_MOTOR')
    else:
      dlg = wx.MessageDialog(self, 'Test already in Progress','Error', wx.OK|wx.ICON_ERROR)
      dlg.ShowModal()
    dlg.Destroy()

  def stop(self, event):
    end_test(self.pid)
    dlg = wx.MessageDialog(self, 'Ending Test','User Hault', wx.OK|wx.ICON_HAND)
    self.testing=0
    dlg.ShowModal()
    
  def check_motor(self, motor):
    if motor[2:7] == '03009':
      return 1
    else:
      return 0 
    
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
    
  def error (self, error_code):
    self.text.SetDefaultStyle(wx.TextAttr(wx.RED))
    if error_code == 'NOT_MOTOR':
      self.text.AppendText("ERROR: The object you scanned is not a motor or the motor can not be evaluated by this test.\n")
    else:
      self.text.AppendText("ERROR: Unknown Error\n")
        
class RosApp(wx.App):
  def OnInit(self):
    frame = RosFrame(None, -1)
    frame.Show(True)
    frame.Centre()
    return True

def set_controller(controller, command):
    s = rospy.ServiceProxy(controller + '/set_command', SetCommand)
    resp = s.call(SetCommandRequest(command))


def spawn_controller(args):
  type, name = args[0], args[1]
  s = rospy.ServiceProxy('spawn_controller', SpawnController)
  resp = s.call(SpawnControllerRequest(type, name, sys.stdin.read()))
  return resp.ok 

def kill_controller(name):
  s = rospy.ServiceProxy('kill_controller', KillController)
  resp = s.call(KillControllerRequest(name))
  return resp.ok 

def start_test(motor,frame):
  xmlFile ='WG_'+motor[2:7]+'.xml'
  sub = popen2.Popen3('botherder')
  path=str(packspec.get_pkg_dir('pr2_etherCAT'))
  path=path+'/pr2_etherCAT rteth0 ../../xml/'+xmlFile
  sub = popen2.Popen3(path)
  frame.pid =sub.pid
  time.sleep(2)
  test_routine(motor,frame)
  
def test_routine(motor,frame):
  set_controller('test_controller',.02)
  frame.testPassed('motor test 1')
  
def end_test(pid):
  grp = os.getpgid(pid)
  os.killpg(grp, signal.SIGTERM)
  return 1

if __name__ == '__main__':
  
  app = RosApp(0)
  app.MainLoop()
  
  print 'quit'
  rospy.signal_shutdown('GUI shutdown')


    
