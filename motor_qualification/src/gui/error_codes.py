#! /usr/bin/env python

import wx

def error (frame, error_code):
    frame.text.SetDefaultStyle(wx.TextAttr(wx.RED))
    if error_code == 'NOT_MOTOR':
      frame.text.AppendText("ERROR: The object you scanned is not a motor or the motor can not be evaluated by this test.\n")
    elif error_code == 'NO_TEST':
      frame.text.AppendText("ERROR: Cannot Stop. No test in progress.\n")
    elif error_code == 'NO_HERD':
      frame.text.AppendText("ERROR: Cannot get measurement from test stand. Most likely botherder did not start\n")  
    elif error_code == 'NO_MOVE':
      frame.text.AppendText("ERROR: Motor shaft not moving. Motor maybe unplugged.\n")
    elif error_code == 'NO_ENCODER':
      frame.text.AppendText("ERROR: The motor encoder is not incrementing. Encoder maybe unplugged.\n")   
    elif error_code == 'SLIP':
      frame.text.AppendText("ERROR: The encoders do not match. The test motor encoder is slipping.\n")  
    elif error_code == 'REV_ENCODER':
      frame.text.AppendText("ERROR: The encoder signs do not match. The test motor encoder wiring or sign is reversed.\n")  
    elif error_code == 'REV_MOTOR':
      frame.text.AppendText("ERROR: The encoders are decrementing. The test motor wiring or sign is wrong.\n")    
    else:
      frame.text.AppendText("ERROR: Unknown Error\n")
    return
