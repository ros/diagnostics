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

import os
from xml.dom import minidom

class NotADirectoryError(Exception): pass
class TestDoesNotExistError(Exception): pass
class FailedLoadError(Exception): pass

class SubTest:
  def __init__(self, subtest, pre_test=None, post_test=None, name=None):
    self._test_script = subtest
    self._pre_script = pre_test
    self._post_script = post_test
    self._name = name

  def get_key(self):
    key = self._test_script
    if self._pre_script is not None:
      key = key + '_' + self._pre_script 
    if self._post_script is not None:
      key = key + '_' + self._post_script
    return key

  def get_name(self):
    if self._name is None:
      return self.get_key()
    else:
      return self._name

      
# Make this load from list/set of subtests
class Test:
  def load(self, test_str):
    self._startup_script = None
    self._shutdown_script = None
    self._instructions_file = None
    self.pre_startup_scripts = []
    self.subtests = []
    self.pre_subtests = {}
    self.post_subtests = {}

    try:
      self._doc = minidom.parseString(test_str)
    except IOError:
      raise FailedLoadError
    
    doc = self._doc
    
    startups = doc.getElementsByTagName('pre_startup')
    if (startups != None and len(startups) > 0):
      for startup in startups:
        name = startup.childNodes[0].nodeValue
        self.pre_startup_scripts.append(name)
        
    elems = doc.getElementsByTagName('startup')
    if (elems != None and len(elems) > 0):
      self._startup_script = elems[0].childNodes[0].nodeValue
    
    elems = doc.getElementsByTagName('shutdown')
    if (elems != None and len(elems) > 0):
      self._shutdown_script = elems[0].childNodes[0].nodeValue
      
    elems = doc.getElementsByTagName('instructions')
    if (elems != None and len(elems) > 0):
      self._instructions_file = elems[0].childNodes[0].nodeValue
    
    subtests = doc.getElementsByTagName('subtest')
    if (subtests != None and len(subtests) > 0):
      for st in subtests:
        script = st.childNodes[0].nodeValue
        pre = None
        post = None
        name = None
        if (st.attributes.has_key('post')):
          post = st.attributes['post'].value
        if (st.attributes.has_key('pre')):
          pre = st.attributes['pre'].value
        if (st.attributes.has_key('name')):
          name = st.attributes['name'].value
        self.subtests.append(SubTest(script, pre, post, name))
 
  def getStartupScript(self):
    return self._startup_script
  
  def getShutdownScript(self):
    return self._shutdown_script
  
  def getInstructionsFile(self):
    return self._instructions_file
  
  
