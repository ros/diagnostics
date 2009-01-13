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
# Revision $Id: gossipbot.py 1013 2008-05-21 01:08:56Z sfkwc $

## Simple demo of a rospy service that add two integers

PKG = 'qualification' # this package name
NAME = 'mcb_conf_verification'

import rostools
rostools.update_path(PKG) 
import wx 
from std_srvs.srv import *
import rospy 
import time

app = wx.PySimpleApp()
process_done = False
prompt_done = False
prompt_click ="no"
frame=wx.Frame(None)

def msg_prompt(msg):
  print "msg_prompt running"
  dlg=wx.MessageBox(msg,"", wx.YES_NO)
  print "a",dlg, wx.YES
  global prompt_done, prompt_click
  if (dlg == wx.YES ):
    prompt_click="yes"
  else:
    prompt_click="no"
  print "b"
  prompt_done=True

  print "msg_prompt done"
  
  
def check_w_user(req):
  print "Result: %s"%req.str
  if req.str == "done":
    wx.CallAfter(frame.Close)
    return StringStringResponse("na")
  global prompt_done
  prompt_done=False
  print "about to call call after" 
  wx.CallAfter(msg_prompt,req.str)
  print "called call after"
  while(not prompt_done):
    print "waiting for prompt . . ."
    time.sleep(1) 
  print prompt_click
  
  if prompt_click =="yes":
    return StringStringResponse("retry")
  else:
    wx.CallAfter(frame.Close)      
    return StringStringResponse("fail")
  
    

def confirm_conf():
  rospy.init_node(NAME)
  s = rospy.Service('mcb_conf_results', StringString, check_w_user)  
  app.MainLoop()
  time.sleep(1)
  



if __name__ == "__main__":  
  confirm_conf()
