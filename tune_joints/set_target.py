#!/usr/bin/python

import roslib; roslib.load_manifest("tune_joints")
import rospy
from std_msgs.msg import Float64
import sys

if len(sys.argv) < 2:
  print "Usage: set_target <controller_name>"
  quit()
controller_name = sys.argv[1]

print 'Sending position commands to "/%s/set_command"\n'%(controller_name)

rospy.init_node('set_target_%s'%(controller_name))

pub = rospy.Publisher("/%s/set_command"%(controller_name), Float64)

while not rospy.is_shutdown():
  try:
    effort = float(raw_input("Enter target location for %s\n"%(controller_name)))
    pub.publish(Float64(effort))
  except:
    print "Error reading input - please enter a number"

print "Exiting\n"

