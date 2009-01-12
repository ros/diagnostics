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

class Test:
  def load(self, dir):
    self._startup_script = None
    self._shutdown_script = None
    self._instructions_file = None
    self.pre_startup_scripts = []
    self.subtests = []
    self.pre_subtests = {}
    self.post_subtests = {}

    if (not os.path.isdir(dir)):
      raise NotADirectoryError
    
    self._test_dir = dir
    self._test_file = os.path.join(dir, 'test.xml')
    if (not os.path.isfile(self._test_file)):
      raise TestDoesNotExistError
    
    try:
      self._doc = minidom.parse(self._test_file)
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
      for subtest in subtests:
        name = subtest.childNodes[0].nodeValue
        self.subtests.append(name)
        if (subtest.attributes.has_key('post')):
          self.post_subtests[name] = subtest.attributes['post'].value
        if (subtest.attributes.has_key('pre')):
          self.pre_subtests[name] = subtest.attributes['pre'].value
  
  def getStartupScript(self):
    return self._startup_script
  
  def getShutdownScript(self):
    return self._shutdown_script
  
  def getInstructionsFile(self):
    return self._instructions_file
  
  def getDir(self):
    return self._test_dir
