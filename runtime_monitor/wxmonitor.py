#!/usr/bin/env python
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

## A basic node to listen to and display incoming diagnostic messages using wx

import rostools
rostools.update_path('runtime_monitor')

import sys
import rospy
from robot_msgs.msg import *

import wx
import threading

NAME = 'runtime_monitor'

ID_ABOUT=101
ID_OPEN=102
ID_BUTTON1=110
ID_EXIT=200

class MainWindow(wx.Frame):
        def __init__(self, parent, id, title):
                self.dirname=''
                wx.Frame.__init__(self, parent, wx.ID_ANY, title)
                self.SetBackgroundColour("BLUE")
                #self.control = wx.TextCtrl(self, 1, style=wx.TE_MULTILINE)
                self.CreateStatusBar()
                self.filemenu = wx.Menu()
                self.filemenu.Append(ID_ABOUT, "&About"," About a file to edit")
                self.filemenu.AppendSeparator()
                self.filemenu.Append(ID_EXIT, "E&xit"," Exit the program")
                self.menubar = wx.MenuBar()
                self.menubar.Append(self.filemenu,"&File")
                self.SetMenuBar(self.menubar)
                wx.EVT_MENU(self, ID_ABOUT, self.OnAbout)
                wx.EVT_MENU(self, ID_EXIT, self.OnExit)
                self.sizer2 = wx.BoxSizer(wx.HORIZONTAL)
                self.buttons = [wx.Button(self, id=-1, label='Button')]
                self.buttons.append(wx.Button(self, id=-1, label='Button1'))
                self.buttons.append(wx.Button(self, id=-1, label='Button2'))
                for b in self.buttons:
                        self.sizer2.Add(b,0,wx.EXPAND)

                #self.sizer2.Add(self.control,0,wx.EXPAND)

                self.sizer = wx.BoxSizer(wx.VERTICAL)
                self.components = []
                self.sizer2.Add(self.sizer, 2, wx.EXPAND)
                
                self.SetSizer(self.sizer2)
                self.SetAutoLayout(1)
                self.sizer2.Fit(self)
                self.Show(True)

                self.counter = 1

                self.lock = threading.Lock()
                
        def OnAbout(self, e):
                d = wx.MessageDialog(self, "A sample editor","About ", wx.OK)
                d.ShowModal()
                d.Destroy()

        def OnExit(self, e):
                self.Close(True)
                

        def callback(self, message):
                self.lock.acquire(1)
                print""
                print "New Message at %.1f"%message.header.stamp.to_time()
                for s in message.status:
        ## @TODO process byte level
                        print "Name: %s \nMessage: %s"%(s.name, s.message)
                        for v in s.values:
                                print "   Value: %.2f Label: %s"%(v.value, v.value_label)
                self.SetTitle("New Message at %.1f"%message.header.stamp.to_time())

                self.sizer2.Remove(self.buttons[0])
                self.buttons.pop(0)
                self.buttons.append(wx.Button( self, id=-1, label="Test %d"%self.counter))
                self.sizer2.Add(self.buttons[-1])
                print "added button"
                self.counter = self.counter + 1
        
                self.sizer2.RecalcSizes()
                self.lock.release()
                self.Show()
    
def listener():
        app = wx.PySimpleApp()
        frame = MainWindow(None, -1, "Test Frame")
        rospy.TopicSub("/diagnostics", DiagnosticMessage, frame.callback)
        rospy.ready(NAME, anonymous=True)
        app.MainLoop()
        
        
if __name__ == '__main__':
    try:
        listener()
    except KeyboardInterrupt, e:
        pass
    print "exiting"
