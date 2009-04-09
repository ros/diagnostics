#!/usr/bin/env python
#
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

# Author: Kevin Watts

import roslib
roslib.load_manifest('motorconf_gui')
import rospy
import roslaunch
import roslaunch.pmon

import os
import sys
import datetime
import wx
import time
from wx import xrc
from wx import html

from std_msgs.msg import *
from mechanism_control import mechanism
from robot_srvs.srv import SpawnController, KillController

from robot_msgs.msg import MechanismState

import cStringIO 

class MotorConfFrame(wx.Frame):
    def __init__(self, parent):
        wx.Frame.__init__(self, parent, wx.ID_ANY, "Motor Configuration GUI")

        # Load XRC resource
        xrc_path = os.path.join(roslib.packages.get_pkg_dir('motorconf_gui'), 'xrc/conf_gui.xrc')
        resource = xrc.XmlResource(xrc_path)

        # Robots with ethercat files
        self._robots = { 
            'PRE' : 'pre.launch',
            'PRF' : 'prf.launch',
            'PRG' : 'prg.launch' }

        # Load robot selection dialog
        dialog = resource.LoadDialog(None, 'robot_select_dialog')
                
        robot_box = xrc.XRCCTRL(dialog, 'robot_list_box')
        robot_box.InsertItems(dict.keys(self._robots), 0)
        
        xrc.XRCCTRL(dialog, 'instruct_text').Wrap(350)

        dialog.Layout()
        dialog.Fit()
        
        if (dialog.ShowModal() == wx.ID_OK):
            robot = robot_box.GetStringSelection()
            self._ethercat_script = self._robots[robot]
            
            # Initialize all buttons, etc
            self._root_panel = resource.LoadPanel(self, 'motorconf_panel')
            self._sizer = wx.BoxSizer(wx.VERTICAL)
            self._sizer.Add(self._root_panel, 1, wx.EXPAND|wx.ALIGN_CENTER_VERTICAL)
            self.SetSizer(self._sizer)        
            
            # Panels
            self._control_panel = xrc.XRCCTRL(self._root_panel, 'control_panel')
            self._log_panel = xrc.XRCCTRL(self._root_panel, 'log_panel')
            
            # Buttons with events
            self._conf_button = xrc.XRCCTRL(self._control_panel, 'configure_mcb_button')
            self._conf_button.Bind(wx.EVT_BUTTON, self.on_conf)
            
            self._twitch_button = xrc.XRCCTRL(self._control_panel, 'twitch_button')
            self._twitch_button.Bind(wx.EVT_BUTTON, self.on_twitch)
            
            self._actuator_num_box = xrc.XRCCTRL(self._control_panel, 'actuator_number_box')
            
            xrc.XRCCTRL(self._control_panel, 'instruct_text').Wrap(340)

            # Add actuators to name box
            self.setup_actuators()
            self._actuator_name_box = xrc.XRCCTRL(self._control_panel, 'actuator_list_box')
            self._actuator_name_box.InsertItems(self._actuators, 0)
            
            # Add joints to twitch box
            self.setup_joints()
            self._twitch_list_box = xrc.XRCCTRL(self._control_panel, 'twitch_list_box')
            self._twitch_list_box.InsertItems(dict.keys(self._joints), 0)
            
            # Windows and text boxes
            self._detect_window = xrc.XRCCTRL(self._control_panel, 'detect_window')
            self._log  = xrc.XRCCTRL(self._log_panel, 'log_text')
            
            self.Bind(wx.EVT_CLOSE, self.on_close)
                        
            self.Layout()
            self.Fit()

            # Launch roscore
            self._core_launcher = self.launch_core()
            rospy.init_node("motorconf_GUI", anonymous = True)
            self.start_pr2_etherCAT()
            rospy.Subscriber("/mechanism_state", MechanismState, self.on_mech_state_msg)
            
  
        else: # close or cancel
            sys.exit(0)
            
    def on_close(self, event):
        event.Skip()

        if self._launcher is not None:
            self._launcher.stop()
        
        if self._core_launcher is not None:
            self._core_launcher.stop()

    def launch_core(self):
        self.log('Launching ros core services')
        
        # Create a roslauncher
        config = roslaunch.ROSLaunchConfig()
        config.master.auto = config.master.AUTO_RESTART
        
        launcher = roslaunch.ROSLaunchRunner(config)
        launcher.launch()
        
        return launcher

    def log(self, msg):
        print msg # To command line
        self._log.AppendText(datetime.datetime.now().strftime("%m/%d/%Y %I:%M:%S: ") + msg + "\n")
        self._log.Refresh()
        self._log.Update()

    def start_pr2_etherCAT(self):
        self.log('Starting pr2_etherCAT')
        config = roslaunch.ROSLaunchConfig()
        script = os.path.join(roslib.packages.get_pkg_dir('pr2_alpha'), self._ethercat_script)
        
        # Call subprocess with pr2_etherCAT and script

        self._launcher = None

        try:
            loader = roslaunch.XmlLoader()
            loader.load(script, config)
            
            launcher = roslaunch.ROSLaunchRunner(config)
            # if (listener_obj != None):
            #     launcher.pm.add_process_listener(listener_obj)
            
            launcher.launch()
            self._launcher = launcher
        except roslaunch.RLException, e:
            self.log('Caught expection shutting down pr2_etherCAT.\nException:\n%s' % e)
            self._launcher = None
            
    def stop_pr2_etherCAT(self):
        self.log('Stopping pr2_etherCAT')
        if (self._launcher != None):
            self._launcher.stop()
            self._launcher = None

    def on_conf(self, event):
        # Record selected actuator index, name
        num = self._actuator_num_box.GetValue()
        name = self._actuator_name_box.GetStringSelection()
        
        self.log('Configuring MCB %s as %s.' % (num, name))
        
        self.stop_pr2_etherCAT()
        
        try:
            # Run motorconf with given commands
            path = roslib.packages.get_pkg_dir("ethercat_hardware", True)
            actuator_path = path + "/actuators.conf"
            
            cmd = path + "/motorconf" + " -i eth0 -p -n %s -d %s -a %s"%(name, num, actuator_path)
#retcode = subprocess.call(cmd, shell=True)
            self.log('Attempting to program actuator number %s as %s' % (num, name))
            p = subprocess.Popen(cmd, stdout = subprocess.PIPE, stderr = subprocess.PIPE, shell = True)
            stdout, stderr = p.communicate()
            retcode = p.returncode
            details = 'Ran motorconf. Attempted to program MCB %s with actuator name %s. Return code: %s.\n\n' % (num, name, retcode)
            details = details + 'STDOUT:\n' + stdout + '\n' + 'STDERR:\n' + stderr
            
            if retcode != 0:
                # Popup msg that says it failed
                self.log(details)
            else:
                self.log('Motorconf succeeded. MCB %s programmed as %s' % (num, name))
        except Exception, e:
            self.log('Caught exception attempting to program motor.\nException:\n%s' % e)
            
        self.start_pr2_etherCAT()

    def xml_for_twitch(self, joint):
        return "\
<controller name=\"%s\" type=\"JointEffortControllerNode\">\
<joint name=\"%s\" />\
</controller>" % (joint + '_controller', joint)
        

    def setup_actuators(self):
        # Parse from actuators.conf file?
        self._actuators = [
            # Head and spine
            'torso_lift_motor',
            'head_pan_motor',
            'head_tilt_motor',
            # Alpha 2.0 head tilt motors are different
            'head_tilt_motor_alpha2a',
            'head_tilt_motor_alpha2b',
            'laser_tilt_mount_motor', 
            # Right arm
            'r_shoulder_pan_motor',
            'r_shoulder_lift_motor',
            'r_upperarm_roll_motor',
            'r_elbow_flex_motor',
            'r_wrist_l_motor',
            'r_wrist_r_motor',
            'r_gripper_motor',
            # Left arm
            'l_shoulder_pan_motor',
            'l_shoulder_lift_motor',
            'l_upperarm_roll_motor',
            'l_elbow_flex_motor',
            'l_wrist_l_motor',
            'l_wrist_r_motor',
            'l_gripper_motor',
            # Casters
            'fl_caster_l_wheel_motor',
            'fl_caster_r_wheel_motor',
            'fl_caster_rotation_motor',
            'fr_caster_l_wheel_motor',
            'fr_caster_r_wheel_motor',
            'fr_caster_rotation_motor',
            'bl_caster_l_wheel_motor',
            'bl_caster_r_wheel_motor',
            'bl_caster_rotation_motor',
            'br_caster_l_wheel_motor',
            'br_caster_r_wheel_motor',
            'br_caster_rotation_motor',
            # Alpha 2.0 caster motors
            'fl_caster_l_wheel_motor_alpha2',
            'fl_caster_r_wheel_motor_alpha2',
            'fl_caster_rotation_motor_alpha2',
            'fr_caster_l_wheel_motor_alpha2',
            'fr_caster_r_wheel_motor_alpha2',
            'fr_caster_rotation_motor_alpha2',
            'bl_caster_l_wheel_motor_alpha2',
            'bl_caster_r_wheel_motor_alpha2',
            'bl_caster_rotation_motor_alpha2',
            'br_caster_l_wheel_motor_alpha2',
            'br_caster_r_wheel_motor_alpha2',
            'br_caster_rotation_motor_alpha2' ]
  
    def setup_joints(self):
        self._joints = {
            # Head and spine
            'torso_lift_joint' : 10000,
            'head_pan_joint' : 10,
            'head_tilt_joint' : 25,
            'laser_tilt_mount_joint' : 10,
            # Right arm
            'r_shoulder_pan_joint' : 20,
            'r_shoulder_lift_joint' : 15,
            'r_upperarm_roll_joint' : 15,
            'r_elbow_flex_joint' : 15,
            'r_forearm_roll_joint' : 10,
            'r_wrist_flex_joint' : 10,
            'r_wrist_roll_joint' : 6,
            'r_gripper_l_finger_joint' : 10,
            # Left arm
            'l_shoulder_pan_joint' : 20,
            'l_shoulder_lift_joint' : 15,
            'l_upperarm_roll_joint' : 15,
            'l_elbow_flex_joint' : 15,
            'l_forearm_roll_joint' : 10,
            'l_wrist_flex_joint' : 10,
            'l_wrist_roll_joint' : 6,
            'l_gripper_l_finger_joint' : 10,
            # Casters
            'fl_caster_l_wheel_joint' : 25,
            'fl_caster_r_wheel_joint' : 25,
            'fl_caster_rotation_joint' : 25,
            'fr_caster_l_wheel_joint' : 25,
            'fr_caster_r_wheel_joint' : 25,
            'fr_caster_rotation_joint' : 25,
            'bl_caster_l_wheel_joint' : 25,
            'bl_caster_r_wheel_joint' : 25,
            'bl_caster_rotation_joint' : 25,
            'br_caster_l_wheel_joint' : 25,
            'br_caster_r_wheel_joint' : 25,
            'br_caster_rotation_joint' : 25 }

    def on_twitch(self, event):
        if self._launcher is None:
            return # Can't twitch if pr2_etherCAT isn't up

        joint = self._twitch_box.GetStringSelection()
        
        self.log('Twitching %s' % joint)
        # Launch effort controller for that joint
        rospy.wait_for_service('spawn_controller')
        spawn_controller = rospy.ServiceProxy('spawn_controller', SpawnController)
        kill_controller = rospy.ServiceProxy('kill_controller', KillController)
        resp = spawn_controller(self.xml_for_twitch(joint))
        if len(resp.ok) < 1 or not resp.ok[0]:
            self.log("Failed to spawn effort controller for joint %s" % joint)

        try:
            effort = self._joints[joint]
            pub = rospy.Publisher("/%s_controller/set_command" % joint, Float64)
            
            # Twitch joint
            pub.publish(Float64(effort))
            sleep(0.3)
            pub.publish(Float64(-effort))
            sleep(0.3)
            pub.publish(Float64(0))
            
            kill_controller(joint + '_controller')
        finally:
            self.log('Twitched %s' % joint)
            kill_controller(joint + '_controller')
          

    # Comparison to sort mechanism state messages by velocity
    def vel_cmp(self, a, b):
        return cmp(abs(a[2]), abs(b[2]))

    # Set detect window with mechanism states
    def on_mech_state_msg(self, data):
        try:
            mech_data = [(data.actuator_states[x].device_id, data.actuator_states[x].encoder_count, data.actuator_states[x].encoder_velocity, data.actuator_states[x].name) for x in xrange(len(data.actuator_states))]
            sorted_mech = list(mech_data)
            sorted_mech.sort(self.vel_cmp)
            
            # Fill detection window with mech state
            self._detect_window.Freeze()
            s = cStringIO.StringIO()
            s.write("<html><body>")
            s.write('<table border="1" cellpadding="2" cellspacing="0">')
            s.write('<tr><b><td>ID</td><td>Name</td><td>Count</td><td>Velocity</td></b></tr>')
            for row in mech_data:
                dev_cnt = row[1]
                dev_id = row[0]
                dev_vel = row[2]
                dev_name = row[3]
                
                # Make row bold if velocity is >0
                bold_txt = ''
                bold_txt2 = ''
                if (dev_vel == row[-1][2] and row[-1][2] != 0):
                    bold_txt = '<b>'
                    bold_txt2 = '</b>'

                s.write('<tr>%s<td>%s</td><td>%s</td><td>%s</td><td>%s</td>%s</tr>\n' % (bold_txt, dev_id, dev_name, dev_cnt, dev_vel, bold_txt2) )
        

            s.write('</table>\n</body></html>')
            (x, y) = self._detect_window.GetViewStart()
            self._detect_window.SetPage(s.getvalue())
            self._detect_window.Scroll(x, y)
            
            self._detect_window.Thaw()
        except Exception, e:
            print 'Caught exception on mech state msg parsing.'
            print e

class MotorConfigurationApp(wx.App):
    def OnInit(self):
        self._frame = MotorConfFrame(None)
        self._frame.SetSize(wx.Size(550, 900))
        self._frame.Layout()
        self._frame.Centre()
        self._frame.Show(True)

        return True

if __name__ == '__main__':
    try:
        app = MotorConfigurationApp(0)
        app.MainLoop()
    except Exception, e:
        print e

    print 'Quitting motorconf gui'
