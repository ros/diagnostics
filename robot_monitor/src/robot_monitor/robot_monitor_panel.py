#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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

# Author: Kevin Watts, Josh Faust

PKG = 'robot_monitor'

import roslib; roslib.load_manifest(PKG)

import sys, os
import rospy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import wx
from wx import xrc

import threading, time

from viewer_panel import StatusViewerFrame
from robot_monitor_generated import MonitorPanelGenerated
from message_timeline import MessageTimeline

color_dict = {0: wx.Colour(85, 178, 76), 1: wx.Colour(222, 213, 17), 2: wx.Colour(178, 23, 46), 3: wx.Colour(40, 23, 176)}
error_levels = [2, 3]

def get_nice_name(status_name):
    return status_name.split('/')[-1]

def get_parent_name(status_name):
    return ('/'.join(status_name.split('/')[:-1])).strip()

class StatusItem(object):
    def __init__(self, status):
        self.tree_id = None
        self.warning_id = None
        self.error_id = None
        self.last_level = None
        self.update(status)
        
    def update(self, status):
        self.status = status
        
class State(object):
    def __init__(self):
        self._items = {}
        self._msg = None
        self._has_warned_no_name = False

    def reset(self):
        self._items = {}
        self._msg = None
        
    def get_parent(self, item):
        parent_name = get_parent_name(item.status.name)
        
        if (parent_name not in self._items):
            return None
        
        return self._items[parent_name]
    
    def get_descendants(self, item):
        child_keys = [k for k in self._items.iterkeys() if k.startswith(item.status.name + "/")]
        children = [self._items[k] for k in child_keys]
        return children
    
    def get_items(self):
        return self._items
        
    def update(self, msg):
        removed = []
        added = []
        items = {}
        
        # fill items from new msg, creating new StatusItems for any that don't already exist,
        # and keeping track of those that have been added new
        for s in msg.status:
            # DiagnosticStatus messages without a name are invalid #3806
            if not s.name and not self._has_warned_no_name:
                rospy.logwarn('DiagnosticStatus message with no "name". Unable to add to robot monitor. Message: %s, hardware ID: %s, level: %d' % (s.message, s.hardware_id, s.level))
                self._has_warned_no_name = True

            if not s.name:
                continue
                
            if (len(s.name) > 0 and s.name[0] != '/'):
                s.name = '/' + s.name

            if (s.name not in self._items):
                i = StatusItem(s)
                added.append(i)
                items[s.name] = i
            else:
                i = self._items[s.name]
                i.update(s)
                items[s.name] = i
        
        # find anything without a parent already in the items, and add it as a dummy
        # item
        to_add = []
        dummy_names = []
        for i in items.itervalues():
            parent = i.status.name
            while (len(parent) != 0):
                parent = get_parent_name(parent)
                if (len(parent) > 0 and parent not in items and parent not in dummy_names):
                    pi = None
                    if (parent not in self._items):
                        s = DiagnosticStatus()
                        s.name = parent
                        s.message = ""
                        pi = StatusItem(s)
                    else:
                        pi = self._items[parent]
                        
                    to_add.append(pi)
                    dummy_names.append(pi.status.name)
                  
        for a in to_add:
            if (a.status.name not in items):
                items[a.status.name] = a
                
                if (a.status.name not in self._items):
                    added.append(a)
        
        for i in self._items.itervalues():
            # determine removed items
            if (i.status.name not in items):
                removed.append(i)
        
        # remove removed items
        for r in removed:
            del self._items[r.status.name]
        
        self._items = items
        self._msg = msg
        
        # sort so that parents are always added before children
        added.sort(cmp=lambda l,r: cmp(l.status.name, r.status.name))
        # sort so that children are always removed before parents
        removed.sort(cmp=lambda l,r: cmp(l.status.name, r.status.name), reverse=True)
        
        #added_keys = [a.status.name for a in added]
        #if len(added_keys) > 0: print "Added: ", added_keys
        #removed_keys = [r.status.name for r in removed]
        #if (len(removed_keys) > 0): print "Removed: ", removed_keys
        
        return (added, removed, self._items)

##\brief Monitor panel for aggregated diagnostics (diagnostics_agg)
##
## Displays data from DiagnosticArray diagnostics_agg in a tree structure
## by status name. Names are parsed by '/'. Each status name is given
## an icon by status (ok, warn, error, stale).
## 
## The robot monitor does not store any state, but if it does not get any updates 
## for 3 seconds, it will mark the tree as stale.
class RobotMonitorPanel(MonitorPanelGenerated):
    ##\param parent RobotMonitorFrame : Parent frame
    def __init__(self, parent, rxbag = False):
        MonitorPanelGenerated.__init__(self, parent)

        self._frame = parent
        self._rxbag = rxbag

        self._tree_ctrl.AddRoot("Root")
        self._error_tree_ctrl.AddRoot("Root")
        self._warning_tree_ctrl.AddRoot("Root")
        
        self._tree_ctrl.SetToolTip(wx.ToolTip("Double click an item for details"))
        self._error_tree_ctrl.SetToolTip(wx.ToolTip("Double click an item for details"))
        self._warning_tree_ctrl.SetToolTip(wx.ToolTip("Double click an item for details"))

        # Image list for icons
        image_list = wx.ImageList(16, 16)
        error_id = image_list.AddIcon(wx.ArtProvider.GetIcon(wx.ART_ERROR, wx.ART_OTHER, wx.Size(16, 16)))
        warn_id = image_list.AddIcon(wx.ArtProvider.GetIcon(wx.ART_WARNING, wx.ART_OTHER, wx.Size(16, 16)))
        ok_id = image_list.AddIcon(wx.ArtProvider.GetIcon(wx.ART_TICK_MARK, wx.ART_OTHER, wx.Size(16, 16)))
        stale_icon = wx.Icon(os.path.join(roslib.packages.get_pkg_dir(PKG), 'icons/stale.png'), wx.BITMAP_TYPE_PNG)
        stale_id = image_list.AddIcon(stale_icon)
        self._tree_ctrl.SetImageList(image_list)
        self._error_tree_ctrl.SetImageList(image_list)
        self._warning_tree_ctrl.SetImageList(image_list)
        self._image_list = image_list
        
        # Tell users we don't have any items yet
        self._image_dict = { 0: ok_id, 1: warn_id, 2: error_id, 3: stale_id }

        # Bind double click event
        self._tree_ctrl.Bind(wx.EVT_TREE_ITEM_ACTIVATED, self.on_all_item_activate)
        self._error_tree_ctrl.Bind(wx.EVT_TREE_ITEM_ACTIVATED, self.on_error_item_activate)
        self._warning_tree_ctrl.Bind(wx.EVT_TREE_ITEM_ACTIVATED, self.on_warning_item_activate)
        self._viewers = {}

        self._state = State()

        # Reset monitor to starting configuration
        self._have_message = False
        self._empty_id = None
        self.reset_monitor()

        # Show stale with timer
        self._timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self._update_message_state)
        self._timer.Start(1000)

        self._timeline = MessageTimeline(self, 30, "diagnostics_agg", DiagnosticArray, self.new_message, self.get_color_for_message, self._on_pause)
        self._timeline.set_message_receipt_callback(self._on_new_message_received)
        self.GetSizer().Add(self._timeline, 0, wx.EXPAND)
        
        # unfortunately tooltips do not work on static text, so this information has to go into the docs only
        #self._message_status_text.SetToolTip(wx.ToolTip("""Shows the status of the received aggregated diagnostic message.

#If the robot monitor has not received a diagnostic message in more than 10 seconds, this will show an error and the background of the Errors/Warnings/All views will turn grey."""))

    ##\brief Sets robot monitor messages status in default start configuration
    def _set_initial_message_state(self):
        if self._rxbag:
            self._message_status_text.SetLabel("No message received")
            self._message_status_text.SetForegroundColour(color_dict[1])
            self._tree_ctrl.SetBackgroundColour(wx.LIGHT_GREY)
            self._error_tree_ctrl.SetBackgroundColour(wx.LIGHT_GREY)
            self._warning_tree_ctrl.SetBackgroundColour(wx.LIGHT_GREY)
        else:
            self._message_status_text.SetLabel("No message received")
            self._message_status_text.SetForegroundColour(color_dict[2])
            self._tree_ctrl.SetBackgroundColour(wx.LIGHT_GREY)
            self._error_tree_ctrl.SetBackgroundColour(wx.LIGHT_GREY)
            self._warning_tree_ctrl.SetBackgroundColour(wx.LIGHT_GREY)

    ##\brief Updates status bar with status of diagnostics_agg topic
    def _update_message_state(self, event = None):
        if not self._have_message:
            self._set_initial_message_state()
            return
        if self._rxbag:
            self._message_status_text.SetLabel("Receiving rxbag messages")
            self._message_status_text.SetForegroundColour(color_dict[0])
            self._tree_ctrl.SetBackgroundColour(wx.WHITE)
            self._error_tree_ctrl.SetBackgroundColour(wx.WHITE)
            self._warning_tree_ctrl.SetBackgroundColour(wx.WHITE)
            self._is_stale = False
            return # Return so we don't call get_time

        current_time = rospy.get_time()
        time_diff = current_time - self._last_message_time
        if (time_diff > 10.0):
            self._message_status_text.SetLabel("Last message received %s seconds ago"%(int(time_diff)))
            self._message_status_text.SetForegroundColour(color_dict[2])
            self._tree_ctrl.SetBackgroundColour(wx.LIGHT_GREY)
            self._error_tree_ctrl.SetBackgroundColour(wx.LIGHT_GREY)
            self._warning_tree_ctrl.SetBackgroundColour(wx.LIGHT_GREY)
            self._is_stale = True
        else:
            seconds_string = "seconds"
            if (int(time_diff) == 1):
                seconds_string = "second"
            self._message_status_text.SetLabel("Last message received %s %s ago"%(int(time_diff), seconds_string))
            self._message_status_text.SetForegroundColour(color_dict[0])
            self._tree_ctrl.SetBackgroundColour(wx.WHITE)
            self._error_tree_ctrl.SetBackgroundColour(wx.WHITE)
            self._warning_tree_ctrl.SetBackgroundColour(wx.WHITE)
            self._is_stale = False
        
    
    ## \brief Called whenever a new message is received by the timeline.  Different from new_message in that it
    ## is called even if the timeline is paused, and only when a new message is received, not when the timeline
    ## is scrubbed
    def _on_new_message_received(self, msg):
        self._last_message_time = rospy.get_time()

    ##\brief Clears messages at startup, or for rxbag plugin
    def reset_monitor(self):
        # Reset tree control
        if self._have_message:
            self._tree_ctrl.DeleteChildren(self._tree_ctrl.GetRootItem())
            self._error_tree_ctrl.DeleteChildren(self._error_tree_ctrl.GetRootItem())
            self._warning_tree_ctrl.DeleteChildren(self._warning_tree_ctrl.GetRootItem())
            self._state.reset()

        if not self._empty_id:
            self._empty_id = self._tree_ctrl.AppendItem(self._tree_ctrl.GetRootItem(), "No data")
            self._tree_ctrl.SetItemImage(self._empty_id, self._image_dict[3])        

        self._is_stale = True

        self._have_message = False

        self._last_message_time = 0.0
        self._have_message = False

        self._set_initial_message_state()

    ##\brief processes new messages, updates tree control
    ##
    ## New messages clear tree under the any names in the message. Selected
    ## name, and expanded nodes will be expanded again after the tree clear.
    ## 
    def new_message(self, msg):
        self._tree_ctrl.Freeze()
        
        # Since we have message, remove empty item
        if not self._have_message:
            self._have_message = True
            self._tree_ctrl.Delete(self._empty_id)
            self._empty_id = None
            #wx.CallAfter(self._update_message_state)
            
        (added, removed, all) = self._state.update(msg)
        
        for a in added:
            self._create_tree_item(a)
            
        for r in removed:
            if (r.tree_id is not None):
                self._tree_ctrl.Delete(r.tree_id)
            if (r.error_id is not None):
                self._error_tree_ctrl.Delete(r.error_id)
            if (r.warning_id is not None):
                self._warning_tree_ctrl.Delete(r.warning_id)
        
        # Update viewers
        for k,v in self._viewers.iteritems():
            if (all.has_key(k)):
                v.set_status(all[k].status)
        
        self._update_status_images()
        
        self._update_error_tree()
        self._update_warning_tree()
        
        self._update_labels()
            
        self._tree_ctrl.Thaw()
        
    def _on_pause(self, paused):
        if (not paused and len(self._viewers) > 0):
            msgs = self._timeline.get_messages()
            states = []
            for msg in msgs:
                state = State()
                state.update(msg)
                states.append(state)
        
        for v in self._viewers.itervalues():
            if (paused):
                v.disable_timeline()
            else:
                v.enable_timeline()    
                for state in states:
                    all = state.get_items()
                    if (all.has_key(v.get_name())):
                        v.set_status(all[v.get_name()].status)
        
    def _update_error_tree(self):
        for item in self._state.get_items().itervalues():
            level = item.status.level
            if (level not in error_levels and item.error_id is not None):
                self._error_tree_ctrl.Delete(item.error_id)
                item.error_id = None
            elif (level in error_levels and item.error_id is None):
                item.error_id = self._error_tree_ctrl.AppendItem(self._error_tree_ctrl.GetRootItem(), item.status.name)
                self._error_tree_ctrl.SetItemImage(item.error_id, self._image_dict[level])
                self._error_tree_ctrl.SetPyData(item.error_id, item)
                
        self._error_tree_ctrl.SortChildren(self._error_tree_ctrl.GetRootItem())
                
    def _update_warning_tree(self):
        for item in self._state.get_items().itervalues():
            level = item.status.level
            if (level != 1 and item.warning_id is not None):
                self._warning_tree_ctrl.Delete(item.warning_id)
                item.warning_id = None
            elif (level == 1 and item.warning_id is None):
                item.warning_id = self._warning_tree_ctrl.AppendItem(self._warning_tree_ctrl.GetRootItem(), item.status.name)
                self._warning_tree_ctrl.SetItemImage(item.warning_id, self._image_dict[level])
                self._warning_tree_ctrl.SetPyData(item.warning_id, item)
                
        self._warning_tree_ctrl.SortChildren(self._warning_tree_ctrl.GetRootItem())
        
    def _update_status_images(self):
        for item in self._state.get_items().itervalues():
            if (item.tree_id is not None):
                level = item.status.level
                
                if (item.status.level != item.last_level):
                    self._tree_ctrl.SetItemImage(item.tree_id, self._image_dict[level])
                    item.last_level = level
    
    def _update_labels(self):
        for item in self._state.get_items().itervalues():
            children = self._state.get_descendants(item)
            errors = 0
            warnings = 0
            for child in children:
                if (child.status.level == 2):
                    errors = errors + 1
                elif (child.status.level == 1):
                    warnings = warnings + 1
            
            base_text = "%s : %s"%(get_nice_name(item.status.name), item.status.message)
            errwarn_text = "%s : %s"%(item.status.name, item.status.message)
            
            if (item.tree_id is not None):
                text = base_text
                if (errors > 0 or warnings > 0):
                    text = "(E: %s, W: %s) %s"%(errors, warnings, base_text)
                self._tree_ctrl.SetItemText(item.tree_id, text)
            if (item.error_id is not None):
                self._error_tree_ctrl.SetItemText(item.error_id, errwarn_text)
            if (item.warning_id is not None):
                self._warning_tree_ctrl.SetItemText(item.warning_id, errwarn_text)
                
          
    def _create_tree_item(self, item):
        # Find parent
        parent = self._state.get_parent(item)
        
        parent_id = self._tree_ctrl.GetRootItem()
        if (parent is not None):
            parent_id = parent.tree_id
        
        ## Add item to tree as short name
        short_name = get_nice_name(item.status.name)
        id = self._tree_ctrl.AppendItem(parent_id, short_name)
        item.tree_id = id
        self._tree_ctrl.SetPyData(id, item)
        self._tree_ctrl.SortChildren(parent_id)

    ##\brief Removes StatusViewerFrame from list to update
    ##\param name str : Status name to remove from dictionary
    def remove_viewer(self, name):
        if self._viewers.has_key(name):
            del self._viewers[name]

    def on_all_item_activate(self, event):
        self._on_item_activate(event, self._tree_ctrl)
        
    def on_error_item_activate(self, event):
        self._on_item_activate(event, self._error_tree_ctrl)
        
    def on_warning_item_activate(self, event):
        self._on_item_activate(event, self._warning_tree_ctrl)
        
    def _on_item_activate(self, event, tree_ctrl):
        id = event.GetItem()
        if id == None:
            event.Skip()
            return

        if tree_ctrl.ItemHasChildren(id):
            tree_ctrl.Expand(id)

        item = tree_ctrl.GetPyData(id)
        if not (item and item.status):
            event.Skip()
            return

        name = item.status.name
        
        if (self._viewers.has_key(name)):
            self._viewers[name].Raise()
        else:
            title = get_nice_name(name)
            
            ##\todo Move this viewer somewhere useful
            viewer = StatusViewerFrame(self._frame, name, self, title)
            viewer.SetSize(wx.Size(500, 600))
            viewer.Layout()
            viewer.Center()
            viewer.Show(True)
            viewer.Raise()
    
            self._viewers[name] = viewer
    
            if (self._timeline.is_paused() or self._rxbag):
                viewer.disable_timeline()
                viewer.set_status(item.status)
            else:
                msgs = self._timeline.get_messages()
                states = []
                for msg in msgs:
                    state = State()
                    state.update(msg)
                    states.append(state)
                    
                for state in states:
                    all = state.get_items()
                    if (all.has_key(item.status.name)):
                        viewer.set_status(all[item.status.name].status)
                        
            

    ##\brief Gets the state of the "top level" diagnostics
    ##
    ## Returns the highest value of any of the root tree items
    ##\return -1 = No diagnostics yet, 0 = OK, 1 = Warning, 2 = Error, 3 = All Stale
    def get_top_level_state(self):
        if (self._is_stale):
            return 3
        
        level = -1
        min_level = 255

        if len(self._state.get_items()) == 0:
            return level

        for item in self._state.get_items().itervalues():
            # Only look at "top level" items
            if self._state.get_parent(item) is not None:
                continue

            if item.status.level > level:
                level = item.status.level
            if item.status.level < min_level:
                min_level = item.status.level
              
        # Top level is error if we have stale items, unless all stale
        if level > 2 and min_level <= 2:
            level = 2

        return level

    def get_color_for_message(self, msg):
        level = 0
        min_level = 255
        
        lookup = {}
        for status in msg.status:
            lookup[status.name] = status
            
        names = [status.name for status in msg.status]
        names = [name for name in names if len(get_parent_name(name)) == 0]
        for name in names:
            status = lookup[name]
            if (status.level > level):
                level = status.level
            if (status.level < min_level):
                min_level = status.level

        # Stale items should be reported as errors unless all stale
        if (level > 2 and min_level <= 2):
            level = 2

                
        return color_dict[level]
