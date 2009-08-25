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

# Author: Kevin Watts

PKG = 'robot_monitor'

import roslib; roslib.load_manifest(PKG)

import sys, os
import rospy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import wx
from wx import xrc

import threading, time

from robot_monitor.viewer_panel import StatusViewerFrame


##\brief Finds list of first-parent names in msg array
##\param msg DiagnosticArray : Received array of DiagnosticStatus messages
def find_parent_names_in_msg(msg):
    parents = []
    for status in msg.status:
        parents.append('/'.join(status.name.split('/')[0:2]))
    parents = set(parents) # Eliminate duplicates
    return parents

##\brief Stores DiagnosticStatus message for all items in tree
class TreeItem(object):
    ##\brief status DiagnosticStatus : Message to store
    ##\brief tree_id wxTreeItemId : Id of tree item
    def __init__(self, status, tree_id):
        self.status = status
        self.tree_id = tree_id
        self.update_time = rospy.get_time()

##\brief Monitor panel for aggregated diagnostics (/diagnostics_agg)
##
## Displays data from DiagnosticArray /diagnostics_agg in a tree structure
## by status name. Names are parsed by '/'. Each status name is given
## an icon by status (ok, warn, error, stale). 
## 
## When new messages arrive, all names in the tree control below that name 
## are deleted. However, two messages with separate first names (ex: 
## '/PRF/...' and '/PRG/...') will not conflict. First names like 'PRF' and 
## 'PRG' in the above example are known as 'first_parent' names throughout
## the class.
class RobotMonitorPanel(wx.Panel):
    ##\param parent RobotMonitorFrame : Parent frame
    def __init__(self, parent):
        wx.Panel.__init__(self, parent, wx.ID_ANY)

        self._frame = parent
        
        self._mutex = threading.Lock()

        xrc_path = os.path.join(roslib.packages.get_pkg_dir(PKG), 'xrc/gui.xrc')
        self._res = xrc.XmlResource(xrc_path)
        self._panel = self._res.LoadPanel(self, 'monitor_panel')
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self._panel, 1, wx.EXPAND)
        self.SetSizer(sizer)

        self._tree_ctrl = xrc.XRCCTRL(self._panel, 'tree_ctrl')
        self._root_id = self._tree_ctrl.AddRoot("Root")
        self._parent_name_to_id = {} # Parent names by id
        self._parent_to_name_to_id = {} # id = dict[parent][name]

        # Image list for icons
        image_list = wx.ImageList(16, 16)
        error_id = image_list.AddIcon(wx.ArtProvider.GetIcon(wx.ART_ERROR, wx.ART_OTHER, wx.Size(16, 16)))
        warn_id = image_list.AddIcon(wx.ArtProvider.GetIcon(wx.ART_WARNING, wx.ART_OTHER, wx.Size(16, 16)))
        ok_id = image_list.AddIcon(wx.ArtProvider.GetIcon(wx.ART_TICK_MARK, wx.ART_OTHER, wx.Size(16, 16)))
        stale_id = image_list.AddIcon(wx.ArtProvider.GetIcon(wx.ART_QUESTION, wx.ART_OTHER, wx.Size(16, 16)))
        self._tree_ctrl.AssignImageList(image_list)

        self._image_dict = { 0: ok_id, 1: warn_id, 2: error_id, 3: stale_id }

        # Bind double click event
        self._tree_ctrl.Bind(wx.EVT_TREE_ITEM_ACTIVATED, self.on_item_active)
        self._viewers = {}

        # Bind key down event for delete
        self._tree_ctrl.Bind(wx.EVT_TREE_KEY_DOWN, self.on_item_key_down)

        # Show stale with timer
        self._timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self._assign_tree_status_images)
        self._timer.Start(3000)
        
        self._subscriber = rospy.Subscriber('/diagnostics_agg', DiagnosticArray,
                                            self.callback)
        self._msg = None

    ##\brief Unregisters subscription from master
    def __del__(self):
        self._subscriber.unregister()
    
    ##\brief Callback for /diagnostics_agg subscription
    def callback(self, msg):
        self._mutex.acquire()
        self._msg = msg
        self._mutex.release()
        wx.CallAfter(self.new_message)

    ##\brief processes new messages, updates tree control
    ##
    ## New messages clear tree under the any names in the message. Selected
    ## name, and expanded nodes will be expanded again after the tree clear.
    ## 
    def new_message(self):
        self._mutex.acquire()

        # Find selected item to reselect next round
        selected_name = None
        id = self._tree_ctrl.GetSelection()
        if id is not None:
            item = self._tree_ctrl.GetPyData(id)
            if item is not None and item.status is not None:
                selected_name = item.status.name

        # Search for first parent names in this message
        parents = find_parent_names_in_msg(self._msg)

        # Find expanded items under first parents
        # Delete any children under parents
        # Reset parent names
        expanded_names = {}
        for parent in parents:
            expanded_names[parent] = []
            self._parent_to_name_to_id[parent] = {}
            if self._parent_name_to_id.has_key(parent):
                self._find_expanded(self._parent_name_to_id[parent], expanded_names[parent])
                self._tree_ctrl.DeleteChildren(self._parent_name_to_id[parent])
                self._tree_ctrl.SetItemText(self._parent_name_to_id[parent], parent.split('/')[-1])

        # Add each status to tree, parents first
        for status in self._msg.status:
            name = status.name
            if not name.startswith('/'):
                name = '/' + name

            first_parent = '/'.join(name.split('/')[0:2]) ## Ex: '/robot'

            # Update item if we have it already in the tree
            if self._parent_to_name_to_id[first_parent].has_key(name):
                self._update_item(self._parent_to_name_to_id[first_parent][name], status)
                continue

            # Adding first parent to tree is special, since we need to add it
            # to the _parent_names_to_ids, _parent_to_name_to_id dictionaries
            if not self._parent_name_to_id.has_key(first_parent):
                # Add tree item to parent
                id = self._create_item(first_parent, self._root_id, first_parent)
                self._parent_name_to_id[first_parent] = id
                self._parent_to_name_to_id[first_parent][first_parent] = id
            else:
                id = self._parent_name_to_id[first_parent]
 
            # Now add second parents, third parents, etc recursively
            for i in range(3, len(name.split('/')) + 1):
                next_parent_name = '/'.join(name.split('/')[0:i])
                if not self._parent_to_name_to_id[first_parent].has_key(next_parent_name):
                    id = self._create_item(first_parent, id, next_parent_name)
                else:
                    id = self._parent_to_name_to_id[first_parent][next_parent_name]

            # Finally, we add the status message to the item
            self._update_item(id, status)

            # Update viewers watching this item, if any
            if self._viewers.has_key(name):
                self._viewers[name].panel.write_status(status)

        # Expand and sort first parents and any items that were
        # previously expanded 
        for parent in expanded_names.keys():
            for expanded in expanded_names[parent]:
                try:
                    id = self._parent_to_name_to_id[parent][expanded]
                    if id.IsOk():
                        self._tree_ctrl.Expand(id)
                except:
                    pass

        # Set selected to previous selection
        for key in self._parent_to_name_to_id.keys():
            if self._parent_to_name_to_id[key].has_key(selected_name):
                id = self._parent_to_name_to_id[key][selected_name]
                self._tree_ctrl.SelectItem(id)

        # Comb through tree and assign images
        self._assign_tree_status_images()  

        # Set message, count of children
        for parent in parents:
            self._set_count_and_message(self._parent_name_to_id[parent])
                                
        self._mutex.release()

    ##\brief Adds ': MESSAGE (COUNT)' to all tree text
    ##
    ## Recursively adds message and child count to  items. Count
    ## is number of immediate children.
    ##\param tree_id wxTreeItemId : Id to start combing through tree
    def _set_count_and_message(self, tree_id):
        item = self._tree_ctrl.GetPyData(tree_id)

        if item and item.status:
            old_text = self._tree_ctrl.GetItemText(tree_id)
            if not self._tree_ctrl.ItemHasChildren(tree_id):
                new_text = '%s: %s' % (old_text, item.status.message)
            else:
                child_count = self._tree_ctrl.GetChildrenCount(tree_id, False)
                new_text = '%s (%d): %s' % (old_text, child_count, item.status.message)
            self._tree_ctrl.SetItemText(tree_id, new_text)

        if not self._tree_ctrl.ItemHasChildren(tree_id):
            return

        sibling, cookie = self._tree_ctrl.GetFirstChild(tree_id)
        while not rospy.is_shutdown():
            self._set_count_and_message(sibling)
            
            sibling, cookie = self._tree_ctrl.GetNextChild(tree_id, cookie)
            if not sibling.IsOk():
                break
            

    ##\brief Removes StatusViewerFrame from list to update
    ##\param name str : Status name to remove from dictionary
    def remove_viewer(self, name):
        if self._viewers.has_key(name):
            del self._viewers[name]

    ##\brief Double click on item to popup window
    ##
    ## Double click starts StatusViewerFrame with details of status 
    ## message. Viewer updates with new messages when received.
    def on_item_active(self, event):
        id = self._tree_ctrl.GetSelection()
        if id == None:
            event.Skip()
            return

        if self._tree_ctrl.ItemHasChildren(id):
            self._tree_ctrl.Expand(id)

        item = self._tree_ctrl.GetPyData(id)
        if not (item and item.status):
            event.Skip()
            return

        name = item.status.name
        title = name.split('/')[-1]
        
        ##\todo Move this viewer somewhere useful
        viewer = StatusViewerFrame(self._frame, name, self, title)
        viewer.SetSize(wx.Size(500, 600))
        viewer.Layout()
        viewer.Center()
        viewer.Show(True)

        self._viewers[name] = viewer

        viewer.panel.write_status(item.status)        


    ##\brief Key down events, deletes items if delete pressed
    ## 
    ## Process key down events of tree controls
    def on_item_key_down(self, event):
        id = self._tree_ctrl.GetSelection()
        if (id == None):
            event.Skip()
            return
        
        key = event.GetKeyCode()
        
        if (key == wx.WXK_DELETE):
            item = self._tree_ctrl.GetPyData(id)
            if not (item and item.status):
                self._tree_ctrl.Delete(id)
                self.Refresh()
                return
            
            # Find first parent
            first_parent_id = id
            while not rospy.is_shutdown():
                parent = self._tree_ctrl.GetItemParent(first_parent_id)
                if parent == self._root_id:
                    break
                else:
                    first_parent_id = parent
                
            first_parent_item = self._tree_ctrl.GetPyData(first_parent_id)
            if first_parent_item and first_parent_item.status:
                first_parent = first_parent_item.status.name
                if self._parent_to_name_to_id[first_parent].has_key(item.status.name):
                    del self._parent_to_name_to_id[first_parent][item.status.name]
                
            if first_parent == item.status.name and self._parent_name_to_id.has_key(first_parent):
                del self._parent_name_to_id[first_parent]

            self._tree_ctrl.Delete(id)
        else:
            event.Skip()
            
        self.Refresh()



    #\brief Assigns tree status images for entire tree
    #\param event Event (optional) : Can be called by timer
    def _assign_tree_status_images(self, event = None):
        self._assign_status_images(self._root_id)
        
    ##\brief Assigns OK/WARN/ERROR/STALE icons to all items by status level
    ##
    ## Recursively goes through tree to assign images. Assigns stale images 
    ## to anything that has been still for >3.0 seconds
    def _assign_status_images(self, id):
        # Assign to ID
        item = self._tree_ctrl.GetPyData(id)
        level = 0
        if item and item.status:
            level = item.status.level
        # Sets items as stale if >3.0 seconds
        if item and rospy.get_time() - item.update_time > 3.0:
            level = 3
        
        image = self._image_dict[level]
        self._tree_ctrl.SetItemImage(id, image)

        if not self._tree_ctrl.ItemHasChildren(id):
            return

        sibling, cookie = self._tree_ctrl.GetFirstChild(id)
        while not rospy.is_shutdown():
            self._assign_status_images(sibling)
            
            sibling, cookie = self._tree_ctrl.GetNextChild(id, cookie)
            if not sibling.IsOk():
                break
                    

    ## Creates an empty tree item
    ##\param first_parent str : Name of first parent
    ##\param parent_id wxTreeItemId : Id of immediate parent
    ##\param name str : Name of tree item
    def _create_item(self, first_parent, parent_id, name):
        ## Add item to tree as short name
        short_name = name.split('/')[-1]
        
        id = self._tree_ctrl.AppendItem(parent_id, short_name)
        item = TreeItem(None, id)
        self._tree_ctrl.SetPyData(id, item)
        self._parent_to_name_to_id[first_parent][name] = id
        self._tree_ctrl.SortChildren(parent_id)

        return id

    ##\brief Updates tree item with new status message
    ##
    ##\param id wxTreeItemId : Id to update
    ##\param status_msg DiagnosticStatus : New message
    def _update_item(self, id, status_msg):
        item = self._tree_ctrl.GetPyData(id)
        if item:
            item.status = status_msg
            item.update_time = rospy.get_time()


    ## Find all tree nodes expanded by parent node
    ##\param id wxTreeItemId : Id of node to see 
    def _find_expanded(self, id, expanded_names):
        if not self._tree_ctrl.ItemHasChildren(id):
            return

        if self._tree_ctrl.IsExpanded(id):
            #print 'Found expanded:', self._tree_ctrl.GetItemText(id)
            item = self._tree_ctrl.GetPyData(id)
            if item.status:
                name = item.status.name
            else:
                name = '/' + self._tree_ctrl.GetItemText(id)
            
            expanded_names.append(name)

        sibling, cookie = self._tree_ctrl.GetFirstChild(id)
        while not rospy.is_shutdown():
            self._find_expanded(sibling, expanded_names)

            sibling, cookie = self._tree_ctrl.GetNextChild(id, cookie)
            if not sibling.IsOk():
                break
    
    ##\brief Gets the "top level" state of the diagnostics, ie. the highest value of any of the root tree items
    ##\return 0 = OK, 1 = Warning, 2 = Error
    def get_top_level_state(self):
        level = 0
        id, cookie = self._tree_ctrl.GetFirstChild(self._root_id)
        while not rospy.is_shutdown():
            item = self._tree_ctrl.GetPyData(id)
            if item and item.status and item.status.level > level:
                level = item.status.level
            
            id, cookie = self._tree_ctrl.GetNextChild(self._root_id, cookie)
            if not id.IsOk():
                break
              
        return level
