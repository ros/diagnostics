#! /usr/bin/env python

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
roslib.load_manifest('qualification')

import rospy
import roslaunch

class RoslaunchProcessListener(roslaunch.pmon.ProcessListener):
  def __init__(self):
    self._died_badly = False
    
  def has_any_process_died_badly(self):
    return self._died_badly

  def process_died(self, process_name, exit_code):
    status = 'Failed!'
    if exit_code == 0 or exit_code == None:
      status = 'OK'

    print "Process %s died with exit code %s. %s" % (process_name, exit_code, status)
    if exit_code != None and exit_code != 0:
      self._died_badly = True

def launch_core():
    # Create a roslauncher
    config = roslaunch.ROSLaunchConfig()
    config.master.auto = config.master.AUTO_RESTART
    
    launcher = roslaunch.ROSLaunchRunner(config)
    launcher.launch()
    
    return launcher

def launch_script(script, listener):
    # Create a roslauncher
    config = roslaunch.ROSLaunchConfig()

    # Try loading the XML file
    try:
      loader = roslaunch.XmlLoader()
      loader.load(script, config)
      
      # Bring up the nodes
      launcher = roslaunch.ROSLaunchRunner(config)
      if (process_listener_object != None):
        launcher.pm.add_process_listener(process_listener_object)
        
      launcher.launch()
    except roslaunch.RLException, e:
      self.log('Failed to launch roslaunch file %s: %s'%(script, e))
      return None

def configure_component(script):
    try:
        core_launch = launch_core()
        
        power_board_launch = launch_script(os.path.join(CONFIG_DIR, '../scripts/power_cycle.launch'), None)
        if power_board_launch == None:
            return (False, 'Unable to cycle power board! Cannot configure MCB!')
        
        power_board_launch.spin()
        
        # Launch configuration script
        self.log('Starting configuration')
        listener = RoslaunchProcessListener()
        config_launcher = self.launch_script(os.path.join(CONFIG_DIR, script), listener)
        
        if config_launcher is None:
            return (False, "Failed to launch configuration script.")
        
        config_launcher.spin()

        if (listener.has_any_process_died_badly()):
            s = 'Configuration failed! Press OK to cancel.' % script
            wx.MessageBox(s, 'Configuration Failed', wx.OK|wx.ICON_ERROR, self)
            return (False, 'Configuration of %s failed!' % name)
        
        shutdown_launch = self.launch_script(os.path.join(CONFIG_DIR, '../scripts/power_board_disable.launch'), None)
        if (shutdown_launch == None):
            s = 'Could not load roslaunch shutdown script. SHUTDOWN POWER BOARD MANUALLY!'
            wx.MessageBox(s, 'Invalid shutdown script!', wx.OK|wx.ICON_ERROR, self)
            return (False, 'Unable to shut down power board')
        
        shutdown_launch.spin()
    
        shutdown_launch.stop()
        config_launcher.stop()
        power_board_launch.stop()
        
        core_launch.stop()
        
        return (True, 'Configuration successful!')
    except:
        ret_str = 'Caught exception configuring component. %s' % str(e)
        return (False, ret_str)

    
