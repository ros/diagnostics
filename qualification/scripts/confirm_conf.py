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

## Prompts the user with a Y/N message if MCB's aren't configured
# Called from configure/program_mcb.py through the mcb_conf_results service

PKG = 'qualification' 
NAME = 'mcb_conf_verification'

import roslib
roslib.load_manifest(PKG) 
import wx 
from wx import xrc
from deprecated_srvs.srv import *
import rospy 
import time
import os

app = wx.PySimpleApp()
process_done = False
prompt_done = False
prompt_click ="no"
frame=wx.Frame(None)

 
def msg_detail_prompt(msg):
  # Parse msg for details
  msg, sep, details = msg.partition(':::')
  if len(details) < 5:
    details = 'No details available.'#

  # Load MCB conf dialog box from gui.xrc
  xrc_path = os.path.join(roslib.packages.get_pkg_dir('qualification'), 'xrc/gui.xrc')
  xrc_resource = xrc.XmlResource(xrc_path)
  # See if it works to set a parent
  dialog = xrc_resource.LoadDialog(None, 'confirm_conf_dialog')
  # Set text in message text
  xrc.XRCCTRL(dialog, 'message_text').SetLabel(msg)
  xrc.XRCCTRL(dialog, 'message_text').Wrap(300)  
  xrc.XRCCTRL(dialog, 'detail_text').AppendText(details)
  
  dialog.Layout()
  dialog.Fit()

  global prompt_click, prompt_done

  if (dialog.ShowModal() == wx.ID_OK):
    prompt_click = "yes"
  else:
    prompt_click = "no"
  prompt_done = True
  dialog.Destroy()
   
def check_w_user(req):
  rospy.logout("Confirm Conf Result: %s" % req.str)
  if req.str == "done":
    wx.CallAfter(frame.Close)
    return StringStringResponse("na")
  global prompt_done
  prompt_done=False

  wx.CallAfter(msg_detail_prompt, req.str)

  while(not prompt_done):
    rospy.logout("Waiting for retry prompt . . .")
    for i in range(0, 5):
      time.sleep(1) 

  rospy.logout("User result: %s" % prompt_click)
  
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
