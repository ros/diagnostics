#!/usr/bin/env python

# ROS stuff
import rospy,rostools
rostools.update_path('mechanism_control')
from mechanism_control.msg import *
import rostools.packspec as packspec

#Testing utils
import error_codes as error
import commands

#python utils
import time, datetime
import os, signal, popen2
import wx


plot_title='Motor Response'

def test_topic(frame):
  rospy.TopicSub("/mechanism_state", MechanismState, mechanism_callback, frame)
  rospy.ready('motor_qualification', anonymous=True)


def mechanism_callback(mechanism_state, frame):
    frame.mechanism_state=mechanism_state
    if frame.output==1:
      output= str(mechanism_state.time)+'  '+str(mechanism_state.joint_states[0].position)+'  '+str(mechanism_state.joint_states[1].position)+'\n'
      frame.file.write(output)
    frame.data.append((mechanism_state.time, mechanism_state.joint_states[1].velocity))
    frame.data2.append((mechanism_state.time, mechanism_state.joint_states[0].velocity))
    frame.data = frame.data[-20:]
    frame.data2 = frame.data2[-20:]

def scan_check(frame,value):
  if check_motor(value):
    frame.text.SetDefaultStyle(wx.TextAttr(wx.BLACK))
    frame.text.AppendText('Starting Motor Test for : %s \n' % value)
    frame.testing = 1
    time=datetime.datetime.now()
    frame.fileName = value +'_'+time.strftime("%Y-%m-%d_%I:%M:%S")+'.test'
    start_test(value,frame)          
  else:
    error.error(frame,'NOT_MOTOR')

def check_motor(motor):
    if motor[2:7] == '03009':
      return 1
    else:
      return 0 

def start_test(motor,frame):
  frame.motor=motor
  xmlFile ='WG_'+motor[2:7]+'.xml'
  path=str(packspec.get_pkg_dir('pr2_etherCAT'))
  path=path+'/pr2_etherCAT rteth0 ../../xml/'+xmlFile
  sub = popen2.Popen3(path)
  time.sleep(2)
  test_routine(frame)
  
def test_routine(frame):
  frame.file=open(frame.fileName,'w')
  frame.file.write("TEST1\n")
  frame.file.write("Time  EncF  EncT\n")
  frame.output=1
  test1(frame)
  #test2(motor,frame)
  
  frame.testDone()
  frame.file.close()

def test1(frame):
  if(frame.mechanism_state):
    fixt_start = frame.mechanism_state.joint_states[0].position
    test_start = frame.mechanism_state.joint_states[1].position
    commands.set_controller('test_controller',0.01)
    time.sleep(0.5)
    commands.set_controller('test_controller',0.0)
    time.sleep(0.5)
    fixt_end = frame.mechanism_state.joint_states[0].position
    test_end = frame.mechanism_state.joint_states[1].position
    if fixt_end-fixt_start==0 and test_end-test_start==0:
      error.error(frame,'NO_MOVE') 
      frame.testError()  
    elif fixt_end-fixt_start>0 and test_end-test_start==0:
      error.error(frame,'NO_ENCODER')
      frame.testError()
    elif  fixt_end-fixt_start+1<test_end-test_start or fixt_end-fixt_start-1>test_end-test_start:
      error.error(frame,'SLIP')
      frame.testError()
    elif  fixt_end-fixt_start>0 and test_end-test_start<0:
      error.error(frame,'REV_ENCODER')
      frame.testError()  
    elif  fixt_end-fixt_start<0 and test_end-test_start<0:
      error.error(frame,'REV_MOTOR')
      frame.testError()    
    else:
      frame.testPassed('motor test 1')
      frame.file.write("PASSED TEST1\n")
      frame.output=0
  else:
    error.error(frame,'NO_HERD')
    frame.testError()
  return
