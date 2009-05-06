#!/usr/bin/env python
import roslib
roslib.load_manifest('auto_arm_commander')

import time
import rospy
import sys
from std_msgs.msg import Empty
from pr2_mechanism_controllers.srv import *
from pr2_mechanism_controllers.msg import *
from robot_msgs.msg import *

from auto_arm_commander.settler import *
from auto_arm_commander.arm_commander import ArmCommander

from auto_arm_commander.msg_cache import MsgCache

from roslib import rostime


rospy.init_node('auto_arm_commander', sys.argv, anonymous=False)

pub = rospy.Publisher('mechanism_state_settled', MechanismState)

# Initialize Arm Command Stuff
arm_commander = ArmCommander('right_arm_trajectory_controller')
settler = Settler(100)
settler.start('mechanism_state')

joints = arm_commander.get_traj_joint_names()
num_cmds = arm_commander.num_cmds()
for k in range(0,num_cmds) :

    # Command arm into the correct location
    print "Command traj %u" % k
    result = arm_commander.cmd_arm(k)
    print "   Result: %u" % result

    led_settled = False
    arm_settled = False
    repeat_count = 0

    arm_settled = False
    while not arm_settled and not rospy.is_shutdown() :
        arm_stats = settler.get_stats_latest(joints, 100)
        arm_settled = True
        for x in arm_stats.ranges :
            arm_settled = arm_settled and (x < .00000001)
        time.sleep(.25)
        print "    [" + ', '.join( ['%.6f'%x for x in arm_stats.ranges] ) + "]"
    print "   Arm is settled!"        

        # Grab a stereotypical MechanismState that's close to the middle of the interval
    cur_mech_state = arm_stats.seg[len(arm_stats.seg)/2]

    if not rospy.is_shutdown() :
        print "Publishing"
        pub.publish(cur_mech_state)
        print "    [" + ', '.join( ['%.3f'%x for x in arm_stats.ranges] ) + "]"


