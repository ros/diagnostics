#! /usr/bin/env python

import rostools

rostools.update_path('generic_controllers')
rostools.update_path('mechanism_control')

from mechanism_control.srv import *
from generic_controllers.srv import *
from std_srvs.srv import Empty, EmptyRequest

def set_controller(controller, command):
  s = rospy.ServiceProxy(controller + '/set_command', SetCommand)
  resp = s.call(SetCommandRequest(command))
  return

def spawn_controller(filename):
  s = rospy.ServiceProxy('spawn_controller', SpawnController)
  resp = s.call(SpawnControllerRequest(filename))
  return

def kill_controller(name):
  s = rospy.ServiceProxy('kill_controller', KillController)
  resp = s.call(KillControllerRequest(name))
  return resp.ok 
  
def end_test():
  s = rospy.ServiceProxy('shutdown', Empty)
  s.call(EmptyRequest())
  return
