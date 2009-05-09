#!/usr/bin/python

import roslib; roslib.load_manifest("tune_joints")
import rospy
from std_msgs.msg import Float64
from robot_mechanism_controllers.msg import JointTuningMsg
import sys
import time

data = []
started = 0

def receiveTuningMsg(msg):
  global data, started
  data.append(msg)
  started = 1

if len(sys.argv) < 3:
  print "Usage: set_target <controller_name> <target>"
  quit()
controller_name = sys.argv[1]

print 'Sending position commands to "/%s/set_command"\n'%(controller_name)

rospy.init_node('set_target_%s'%(controller_name))

pub = rospy.Publisher("/%s/set_command"%(controller_name), Float64)
sub = rospy.Subscriber("/%s/tuning"%(controller_name), JointTuningMsg, receiveTuningMsg)

while not started:
  time.sleep(0.1)

time.sleep(0.1)
try:
  effort = float(sys.argv[2])
  pub.publish(Float64(effort))
except:
  print "Error reading input - please enter a number"

time.sleep(1)

sub.unregister()
pub.unregister()

from pylab import *

set_point = [t.set_point for t in data]
position = [t.position for t in data]
velocity = [t.velocity for t in data]
torque = [t.torque for t in data]
torque_measured = [t.torque_measured for t in data]
time_step = [t.time_step for t in data]
count = [(t.count - data[0].count) % 2**32 for t in data]
dcount = diff(count)

subplot(4, 1, 1)
plot(count, position)
plot(count, set_point)
subplot(4, 1, 2)
plot(count, velocity)
subplot(4, 1, 3)
plot(count, torque)
plot(count, torque_measured)
subplot(4, 1, 4)
plot(dcount)

show()
