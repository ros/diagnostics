#!/usr/bin/env python
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Melonee Wise

PKG = "joint_qualification_controllers"

import rostools; rostools.update_path(PKG)

import numpy
import math
from robot_msgs.msg import *
from std_msgs.msg import *
from robot_srvs.srv import *

import sys
import os
import string
from time import sleep

import rospy

import matplotlib.pyplot as plot
from StringIO import StringIO

class App:
  def __init__(self):
    self.data_topic = rospy.TopicSub("/test_data", TestData, self.OnData)
    return True
    
  def OnData(self,msg):
    print 'Got data named %s' % (msg.test_name)
    self.data = msg
    if self.data.test_name=="hysteresis":
      self.HysteresisPlot()
    elif self.data.test_name=="sinesweep":
      self.SineSweepPlot()
    else:
      print 'this test message cannot be analyzed'
      
  def HysteresisPlot(self):
    s = self.HysteresisAnalysis()
    print "plotting hysteresis"
    #create the figure
    fig=plot.figure(1)
    axes1 = fig.add_subplot(211)
    axes1.clear()
    axes2 = fig.add_subplot(212)
    axes2.clear()
    axes1.plot(numpy.array(self.data.position), numpy.array(self.data.effort), 'r--')
    #show the average effort lines 
    axes1.axhline(y=self.min_avg,color='b')
    axes1.axhline(y=0,color='k')
    axes1.axhline(y=self.max_avg,color='g')
    axes1.set_xlabel('Position')
    axes1.set_ylabel('Effort')
    #show that a constant velocity was achieved
    axes2.plot(numpy.array(self.data.position), numpy.array(self.data.velocity), 'b--')
    axes2.set_xlabel('Position')
    axes2.set_ylabel('Velocity')
    
    result_service = rospy.ServiceProxy('test_result', TestResult)
    r = TestResultRequest()
    r.plots = []
    r.result = TestResultRequest.RESULT_HUMAN_REQUIRED
    
    stream = StringIO()
    plt.savefig(stream, format="png")
    image = stream.getvalue()
    
    p = qualification.msg.Plot()
    r.plots.append(p)
    p.instructions = s
    p.image = image
    p.image_format = "png"
    result_service.call(r)
    
  def SineSweepPlot(self):
    print "plotting sinesweep"
    # Plot the values and line of best fit
    fig=plot.figure(1)
    axes1 = fig.add_subplot(211)
    axes1.clear()
    axes2 = fig.add_subplot(212)
    axes2.clear()
    next_pow_two=int(2**math.ceil(math.log(numpy.array(self.data.position).size,2)))
    #plot in decibels
    axes1.psd(numpy.array(self.data.effort), NFFT=next_pow_two, Fs=1000, Fc=0, color='r')
    axes1.psd(numpy.array(self.data.position), NFFT=next_pow_two, Fs=1000, Fc=0)
    axes1.set_xlim(0, 100)
    axes1.set_xlabel('Position PSD')
    
    #plot in power
    pxx, f = axes2.psd(numpy.array(self.data.velocity), NFFT=next_pow_two, Fs=1000, Fc=0)
    axes2.clear()
    axes2.plot(f,pxx)
    #find the peak
    index = numpy.argmax(pxx)
    max_value=max(pxx)
    axes2.plot([f[index]],[pxx[index]],'r.', markersize=10);
    self.first_mode = f[index]
    s = self.SineSweepAnalysis()
    axes2.axvline(x=self.data.arg_value[0],color='r')
    axes2.set_xlim(0, 100)
    axes2.set_xlabel('Velocity PSD')
    #axes2.set_ylabel('Velocity')
    self.count=0
    
    result_service = rospy.ServiceProxy('test_result', TestResult)
    r = TestResultRequest()
    r.plots = []
    r.result = TestResultRequest.RESULT_HUMAN_REQUIRED
    
    stream = StringIO()
    plt.savefig(stream, format="png")
    image = stream.getvalue()
    
    p = qualification.msg.Plot()
    r.plots.append(p)
    p.instructions = s
    p.image = image
    p.image_format = "png"
    result_service.call(r)

  def HysteresisAnalysis(self):
    #compute the encoder travel
    min_encoder=min(numpy.array(self.data.position))
    max_encoder=max(numpy.array(self.data.position))
    #find the index to do the average over
    index1 = numpy.argmax(numpy.array(self.data.position))
    index1= numpy.argmin(numpy.array(self.data.position))
    index=min(index1,index2)
    end =numpy.array(self.data.position).size
    #compute the averages to display
    self.min_avg = min(numpy.average(numpy.array(self.data.effort)[0:index]),numpy.average(numpy.array(self.data.effort)[index:end]))
    self.max_avg = max(numpy.average(numpy.array(self.data.effort)[0:index]),numpy.average(numpy.array(self.data.effort)[index:end]))
    
    s = StringIO()
    
    #check to see we went the full distance
    if min_encoder > self.data.arg_value[2] or max_encoder < self.data.arg_value[3]:
      print >> s, "mechanism is binding and not traveling the complete distace"
      print >> s, "min expected: %f  measured: %f" % (self.data.arg_value[2],min_encoder)
      print >> s, "max expected: %f  measured: %f" % (self.data.arg_value[3],max_encoder)
    #check to see we didn't use too much force
    elif abs(self.min_avg-self.data.arg_value[0])/self.data.arg_value[0]>0.1 or abs(self.max_avg-self.data.arg_value[1])/self.data.arg_value[1]>0.1:
      print >> s, "the mechanism average effort is too high"
      print >> s, "min_expected: %f  min_measured : %f" % (self.data.arg_value[0],self.min_avg)
      print >> s, "max_expected: %f  max_measured : %f" % (self.data.arg_value[1],self.max_avg)
    #data looks okay see what the user thinks
    else:
      print >> s, "data reasonable"
      
    return s.getvalue()
    
    
  def SineSweepAnalysis(self):   
    s = StringIO()
    
    if abs(self.first_mode-self.data.arg_value[0])/self.data.arg_value[0]>self.data.arg_value[2]:
      print >> s, "mechanism is binding and not traveling the complete distace"
      print >> s, "first mode expected: %f  measured: %f" % (self.data.arg_value[0],self.first_mode)
    #data looks okay see what the user thinks  
    else:
      print >> s, "data reasonable"
      
    return s.getvalue()
    
if __name__ == "__main__":
  try:
    rospy.init_node("TestPlotter", anonymous=True)
    app = App()
    rospy.spin()
  except Exception, e:
    print e
    
  app.data_topic.unregister()
  print 'quit'
