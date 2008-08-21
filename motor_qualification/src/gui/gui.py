#! /usr/bin/env python
# Provides quick access to the services exposed by MechanismControlNode

import rostools
rostools.update_path('mechanism_control')
rostools.update_path('generic_controllers')
rostools.update_path('std_srvs')

from mechanism_control.srv import *
from generic_controllers.srv import *
from mechanism_control.msg import *
from std_srvs.srv import Empty, EmptyRequest
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
    self.pid=None
    self.testing = 0 
    #self.data = [(1,2), (2,3), (3,5), (4,6), (5,8), (6,8), (10,10)]
    self.data =[(0,0)]
    self.data2 =[(0,0)]
    text1 = wx.StaticText(panel, -1, 'Motor Response')
    sizer.Add(text1, (0, 0), flag=wx.TOP | wx.LEFT | wx.BOTTOM, border=5)


    self.client = plot.PlotCanvas(panel,-1)
    self.drawPlot()
    sizer.Add(self.client, (1, 0), (5, 3), wx.EXPAND | wx.LEFT | wx.RIGHT, 5)                

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
    self.Bind(wx.EVT_IDLE, self.onIdle)
    self.Centre()
  
  def drawPlot(self):
    #markers = plot.PolyMarker(self.data, legend='', colour='blue', marker='circle', size=1)
    markers = plot.PolyLine(self.data)
    markers2 = plot.PolyLine(self.data2, colour='blue')
    gc = plot.PlotGraphics([markers,markers2], 'Motor Data', 'Time', 'Velocity')
    self.client.Draw(gc)
     
  def callback(self, mechanism_state):
    self.data.append((mechanism_state.time, mechanism_state.joint_states[1].velocity))
    self.data2.append((mechanism_state.time, mechanism_state.joint_states[0].velocity))
    self.data = self.data[-100:]
    self.data2 = self.data2[-100:]

  def onIdle(self, event):
    event.RequestMore(True)
    self.drawPlot()
    
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
    if(self.testing):
      end_test()
      dlg = wx.MessageDialog(self, 'Ending Test','User Hault', wx.OK|wx.ICON_HAND)
      self.testing=0
      dlg.ShowModal()
    else:
      self.error('NO_TEST')
    return 
   
    
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
    elif error_code == 'NO_TEST':
      self.text.AppendText("ERROR: Cannot Stop. No test in progress.\n")
    else:
      self.text.AppendText("ERROR: Unknown Error\n")
    return
        
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
  rospy.TopicSub("/mechanism_state", MechanismState, mechanism_callback, frame)
  rospy.ready('motor_qualification', anonymous=True)
  path=str(packspec.get_pkg_dir('pr2_etherCAT'))
  path=path+'/pr2_etherCAT rteth0 ../../xml/'+xmlFile
  sub = popen2.Popen3(path)
  time.sleep(2)
  test_routine(motor,frame)
  
def test_routine(motor,frame):
  set_controller('test_controller',.02)
  frame.testPassed('motor test 1')
  
def test1(motor,frame):
  return
    
  
  
  
def end_test():
  s = rospy.ServiceProxy('shutdown', Empty)
  s.call(EmptyRequest())
  return
  
def mechanism_callback(data, frame):
  frame.callback(data)
 

if __name__ == '__main__':
  sub = popen2.Popen3('botherder')
  time.sleep(1)
  try:
    app = RosApp(0)
    app.MainLoop()
  except:
    pass
    
  os.killpg(os.getpgid(sub.pid), signal.SIGTERM)
    
  print 'quit'
  rospy.signal_shutdown('GUI shutdown')


    
