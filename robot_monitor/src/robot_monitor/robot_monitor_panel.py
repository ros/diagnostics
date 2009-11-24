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

# Author: Kevin Watts, Josh Faust

PKG = 'robot_monitor'

import roslib; roslib.load_manifest(PKG)

import sys, os
import rospy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import wx
from wx import xrc

import threading, time

from robot_monitor.viewer_panel import StatusViewerFrame

def get_nice_name(status_name):
    return status_name.split('/')[-1]

def get_parent_name(status_name):
    return ('/'.join(status_name.split('/')[:-1])).strip()

class StatusItem(object):
    def __init__(self, status):
        self.tree_id = None
        self.warning_id = None
        self.error_id = None
        self.update(status)
        
    def update(self, status):
        self.status = status
        self.update_time = rospy.get_time()
        
class State(object):
    def __init__(self):
        self._items = {}
        self._msg = None
        
    def get_parent(self, item):
        parent_name = get_parent_name(item.status.name)
        
        if (parent_name not in self._items):
            return None
        
        return self._items[parent_name]
    
    def get_children(self, item):
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
        for i in items.itervalues():
            parent = i.status.name
            while (len(parent) != 0):
                parent = get_parent_name(parent)
                if (len(parent) > 0 and parent not in items):
                    #print "Adding dummy: '%s'"%(parent)
                    s = DiagnosticStatus()
                    s.name = parent
                    s.message = "Missing item"
                    pi = StatusItem(s)
                    to_add.append(pi)
                  
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

##\brief Monitor panel for aggregated diagnostics (/diagnostics_agg)
##
## Displays data from DiagnosticArray /diagnostics_agg in a tree structure
## by status name. Names are parsed by '/'. Each status name is given
## an icon by status (ok, warn, error, stale). The robot monitor will mark an item
## as stale after it is invisible for 3 seconds. Other than that, it does not store
## state. Any item whose parent is updated but that is not updated in the same message
## will be deleted.
## 
## Two messages with separate first names (ex: '/PRF/...' and '/PRG/...') will 
## not conflict and can "share" the robot monitor. First names like 'PRF' and 
## 'PRG' in the above example are known as 'first_parent' names throughout
## the class.
class RobotMonitorPanel(wx.Panel):
    ##\param parent RobotMonitorFrame : Parent frame
    def __init__(self, parent):
        wx.Panel.__init__(self, parent, wx.ID_ANY)

        self._frame = parent

        xrc_path = os.path.join(roslib.packages.get_pkg_dir(PKG), 'xrc/gui.xrc')
        self._res = xrc.XmlResource(xrc_path)
        self._panel = self._res.LoadPanel(self, 'monitor_panel')
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self._panel, 1, wx.EXPAND)
        self.SetSizer(sizer)

        self._tree_ctrl = xrc.XRCCTRL(self._panel, 'tree_ctrl')
        self._tree_ctrl.AddRoot("Root")
        
        self._error_tree_ctrl = xrc.XRCCTRL(self._panel, 'error_tree')
        self._error_tree_ctrl.AddRoot("Root")
        self._warning_tree_ctrl = xrc.XRCCTRL(self._panel, 'warning_tree')
        self._warning_tree_ctrl.AddRoot("Root")

        # Image list for icons
        image_list = wx.ImageList(16, 16)
        error_id = image_list.AddIcon(wx.ArtProvider.GetIcon(wx.ART_ERROR, wx.ART_OTHER, wx.Size(16, 16)))
        warn_id = image_list.AddIcon(wx.ArtProvider.GetIcon(wx.ART_WARNING, wx.ART_OTHER, wx.Size(16, 16)))
        ok_id = image_list.AddIcon(wx.ArtProvider.GetIcon(wx.ART_TICK_MARK, wx.ART_OTHER, wx.Size(16, 16)))
        stale_id = image_list.AddIcon(wx.ArtProvider.GetIcon(wx.ART_QUESTION, wx.ART_OTHER, wx.Size(16, 16)))
        self._tree_ctrl.SetImageList(image_list)
        self._error_tree_ctrl.SetImageList(image_list)
        self._warning_tree_ctrl.SetImageList(image_list)
        self._image_list = image_list
        
        # Tell users we don't have any items yet
        self._empty_id = self._tree_ctrl.AppendItem(self._tree_ctrl.GetRootItem(), "No data")
        self._tree_ctrl.SetItemImage(self._empty_id, stale_id)
        self._have_message = False

        self._image_dict = { 0: ok_id, 1: warn_id, 2: error_id, 3: stale_id }

        # Bind double click event
        self._tree_ctrl.Bind(wx.EVT_TREE_ITEM_ACTIVATED, self.on_all_item_activate)
        self._error_tree_ctrl.Bind(wx.EVT_TREE_ITEM_ACTIVATED, self.on_error_item_activate)
        self._warning_tree_ctrl.Bind(wx.EVT_TREE_ITEM_ACTIVATED, self.on_warning_item_activate)
        self._viewers = {}

        # Show stale with timer
        self._timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self._update_stale_items)
        self._timer.Start(3000)
        
        self._state = State()
        
        self._subscriber = rospy.Subscriber('/diagnostics_agg', DiagnosticArray,
                                            self.callback)

    ##\brief Unregisters subscription from master in destructor
    def __del__(self):
        self._subscriber.unregister()
        
    def _update_stale_items(self, event):
        self._update_status_images()
    
    ##\brief Callback for /diagnostics_agg subscription
    def callback(self, msg):
        wx.CallAfter(self.new_message, msg)

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
                v.panel.write_status(all[k].status)
        
        self._update_status_images()
        
        self._update_error_tree()
        self._update_warning_tree()
        
        self._update_labels()
            
        self._tree_ctrl.Thaw()
        
    def _update_error_tree(self):
        for item in self._state.get_items().itervalues():
            level = item.status.level
            if (level != 2 and item.error_id is not None):
                self._error_tree_ctrl.Delete(item.error_id)
                item.error_id = None
            elif (level == 2 and item.error_id is None):
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
                # Sets items as stale if >3.0 seconds
                if rospy.get_time() - item.update_time > 3.0:
                    level = 3
            
                self._tree_ctrl.SetItemImage(item.tree_id, self._image_dict[level])
    
    def _update_labels(self):
        for item in self._state.get_items().itervalues():
            children = self._state.get_children(item)
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
        title = get_nice_name(name)
        
        ##\todo Move this viewer somewhere useful
        viewer = StatusViewerFrame(self._frame, name, self, title)
        viewer.SetSize(wx.Size(500, 600))
        viewer.Layout()
        viewer.Center()
        viewer.Show(True)

        self._viewers[name] = viewer

        viewer.panel.write_status(item.status)

    ##\brief Gets the "top level" state of the diagnostics
    ##
    ## Returns the highest value of any of the root tree items
    ##\return -1 = No diagnostics yet, 0 = OK, 1 = Warning, 2 = Error, 3 = Stale
    def get_top_level_state(self):
        level = -1
        id, cookie = self._tree_ctrl.GetFirstChild(self._tree_ctrl.GetRootItem())
        if id == self._empty_id:
            return level

        while not rospy.is_shutdown():
            item = self._tree_ctrl.GetPyData(id)
            if item and item.status and item.status.level > level:
                level = item.status.level
            
            id, cookie = self._tree_ctrl.GetNextChild(self._tree_ctrl.GetRootItem(), cookie)
            if not id.IsOk():
                break
              
        return level
