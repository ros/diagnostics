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

PKG = "qualification"

import roslib; roslib.load_manifest(PKG)

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

import matplotlib
matplotlib.use('WXAgg')
import matplotlib.pyplot as plot
from StringIO import StringIO

from qualification.msg import *
from qualification.srv import *

class App:
  def __init__(self):
    rospy.init_node("TestPlotter", anonymous=True)
    self.data_topic = rospy.Service("/test_data", TestData, self.OnData)
    rospy.spin()
    
  def OnData(self,req):
    print 'Got data named %s' % (req.test_name)
    self.data = req
    if self.data.test_name=="hysteresis":
      self.HysteresisPlot()
    elif self.data.test_name=="sinesweep":
      self.SineSweepPlot()
    elif self.data.test_name=="backlash":
      self.BacklashPlot()
    else:
      print 'this test message cannot be analyzed'
    return TestDataResponse(1)
      
  def HysteresisPlot(self):
    s,tr = self.HysteresisAnalysis()

    #create the figure
    fig=plot.figure(1)


    axes1 = fig.add_subplot(211)
    axes2 = fig.add_subplot(212)
    axes1.set_xlabel('Position')
    axes1.set_ylabel('Effort')
    axes2.set_xlabel('Position')
    axes2.set_ylabel('Velocity')
    #plot the effort hysteresis
    axes1.plot(numpy.array(self.data.position), numpy.array(self.data.effort), 'r--')
    #show the average effort lines 
    axes1.axhline(y=self.data.arg_value[1],color='b')
    axes1.axhline(y=0,color='k')
    axes1.axhline(y=self.data.arg_value[0],color='b')
    #show that a constant velocity was achieved
    axes2.plot(numpy.array(self.data.position), numpy.array(self.data.velocity), 'b--')

    # Title from testdata
    fig.text(.35, .95, self.data.joint_name + ' Hysteresis Test')
    
    #pass along results
    result_service = rospy.ServiceProxy('test_result', TestResult)
    r = TestResultRequest()
    r.text_result = ""
    r.plots = []
    if tr == True:
      r.result = TestResultRequest.RESULT_PASS
    else:
      r.result = TestResultRequest.RESULT_HUMAN_REQUIRED
    
    stream = StringIO()
    plot.savefig(stream, format = "png")
    image = stream.getvalue()
    
    p = qualification.msg.Plot()
    r.plots.append(p)
    p.text = s
    p.title = self.data.joint_name + "_hysteresis"
    p.image = image
    p.image_format = "png"
    result_service.call(r)
 
  def HysteresisAnalysis(self):
    # Expected values
    min_effort_param = self.data.arg_value[0]
    max_effort_param = self.data.arg_value[1]
    min_range_param  = self.data.arg_value[2]
    max_range_param  = self.data.arg_value[3]
    
    # Compute the encoder travel
    min_encoder = min(numpy.array(self.data.position))
    max_encoder = max(numpy.array(self.data.position))

    # Find the index to do the average over
    # Data set starts close to one end of hysteresis
    # Find endpoint of travel closest to midpoint of data
    index_max = numpy.argmax(numpy.array(self.data.position))
    index_min = numpy.argmin(numpy.array(self.data.position))
    end = numpy.array(self.data.position).size
    if (abs(end/2-index_max) < abs(end/2-index_min)):
      index = index_max
    else:
      index = index_min

    #compute the average efforts with std deviations to display
    effort1 = numpy.average(numpy.array(self.data.effort)[0:index-15])
    effort2 = numpy.average(numpy.array(self.data.effort)[index+15:end])

    effort1_sd = numpy.std(numpy.array(self.data.effort)[0:index-15])
    effort2_sd = numpy.std(numpy.array(self.data.effort)[index+15:end])

    if effort1 < effort2:
      self.min_avg = effort1
      self.min_sd  = effort1_sd
      self.max_avg = effort2
      self.max_sd  = effort2_sd
    else:
      self.max_avg = effort1
      self.max_sd  = effort1_sd
      self.min_avg = effort2
      self.min_sd  = effort2_sd
    
    #self.max_avg = max(numpy.average(numpy.array(self.data.effort)[0:index-15]),numpy.average(numpy.array(self.data.effort)[index+15:end]))

    tr = True
    range_msg    = "OK"
    negative_msg = "OK"
    positive_msg = "OK"
    error_reason = ""
    
    # Check to range against expected values
    if (min_encoder > min_range_param or max_encoder < max_range_param) and (min_range_param != 0 and max_range_param != 0):
      error_reason = error_reason + "Mechanism is not traveling the complete distance!\n"
      range_msg = "FAIL"
      tr = False

    # Check effort limits (within 15% of expected)
    if abs((self.min_avg - min_effort_param)/min_effort_param) > 0.15:
      error_reason = error_reason +  "The mechanism average effort in the negative direction is out of bounds!\n"
      negative_msg = "FAIL"
      tr = False
    if abs((self.max_avg - max_effort_param)/max_effort_param) > 0.15:
      error_reason = error_reason +  "The mechanism average effort in the positive direction is out of bounds!\n"
      positive_MSG = "FAIL"
      tr = False

    # Check that effort is even (<20% standard deviation)
    if abs(self.min_sd / self.min_avg) > 0.20:
      error_reason = error_reason + "Effort is uneven in negative direction\n"
      negative_msg = "FAIL with uneven effort!"
      tr = False
    if abs(self.max_sd / self.max_avg) > 0.20:
      error_reason = error_reason + "Effort is uneven in positive direction\n"
      positive_msg = "FAIL with uneven effort!"
      tr = False

    if tr:
      test_msg = "FAIL"
    else:
      test_msg = "PASS"

    # Standard deviations in percents
    sd_min_percent = abs(self.min_sd / self.min_avg) * 100
    sd_max_percent = abs(self.max_sd / self.max_avg) * 100

    s = StringIO() 
    print >> s, "Test result: %s" % test_msg
    print >> s, "Efforts."
    print >> s, "Positive. Average: %f, SD: %f percent. Expected: %f. Status: %s" % (self.max_avg, sd_max_percent, max_effort_param, positive_msg)  
    print >> s, "Negative. Average: %f, SD: %f percent. Expected: %f. Status: %s" % (self.min_avg, sd_min_percent, min_effort_param, negative_msg)  
    print >> s, "Joint limits"
    print >> s, "Max measured: %f, expected: %f. Min measured: %f, expected: %f: Status: %s" % (max_encoder, max_range_param, min_encoder, min_range_param, range_msg)
    
    if not tr:
      print >> s, "\n"   
      print >> s, "Test failure data:"
      print >> s, error_reason

    return (s.getvalue(),tr)

  def SineSweepPlot(self):
    # Plot the values and line of best fit
    fig = plot.figure(1)
    axes1 = fig.add_subplot(211)
    axes1.clear()
    axes2 = fig.add_subplot(212)
    axes2.clear()
    # Find the next power of two above our number of data points
    next_pow_two=int(2**math.ceil(math.log(numpy.array(self.data.position).size,2)))
    #plot in decibels
    axes1.psd(numpy.array(self.data.effort), NFFT=next_pow_two, Fs=1000, Fc=0, color='r')
    axes1.psd(numpy.array(self.data.position), NFFT=next_pow_two, Fs=1000, Fc=0)
    axes1.set_xlim(0, 100)
    #axes1.set_xlabel('Frequency')
    axes1.set_title('Position PSD')
    
    #plot in power (pxx - power, f - freqs)
    pxx, f = axes2.psd(numpy.array(self.data.velocity), NFFT=next_pow_two, Fs=1000, Fc=0)
    axes2.clear()
    for i in f:
      if f[i]>4:
        cutoff=i # Set cutoff to avoid finding first mode at extremely low freqs
        break
    axes2.plot(f,pxx)

    #find the peak
    index = numpy.argmax(pxx[cutoff:pxx.size])
    index = index + cutoff
    max_value = max(pxx[cutoff:pxx.size])
    axes2.plot([f[index]],[pxx[index]],'r.', markersize=10);
    self.first_mode = f[index]

    s,tr = self.SineSweepAnalysis()
    axes2.axvline(x=self.data.arg_value[0], color='r') # Line at first mode
    axes2.set_xlim(0, 100)
    axes2.set_ylim(0, max_value+10)
    axes2.set_xlabel('Frequency')
    axes2.set_ylabel('Power')
    axes2.set_title('Velocity PSD')

    # Title
    fig.text(.35, .95, self.data.joint_name + ' SineSweep Test')

    result_service = rospy.ServiceProxy('test_result', TestResult)
    r = TestResultRequest()
    r.text_result = ""
    r.plots = []
    if tr==True:
      r.result = TestResultRequest.RESULT_PASS
    else:
      r.result = TestResultRequest.RESULT_HUMAN_REQUIRED
    
    stream = StringIO()
    plot.savefig(stream, format="png")
    image = stream.getvalue()
    
    p = qualification.msg.Plot()
    r.plots.append(p)
    p.text = s
    p.title = self.data.joint_name + "_sine_sweep"
    p.image = image
    p.image_format = "png"
    result_service.call(r)

  def SineSweepAnalysis(self):   
    # Parameters
    first_mode_param = self.data.arg_value[0]
    mode_error_param = self.data.arg_value[2]

    s = StringIO()
    if abs(self.first_mode - first_mode_param)/first_mode_param > mode_error_param:
      print >> s, "Test Result: FAIL"
      print >> s, "The first mode is incorrect: FAIL"
      tr=False
    else:
      print >> s, "Test Result: PASS"
      print >> s, "First mode within tolerance: OK"
      tr=True
    print >> s, "First mode measured: %f, expected: %f" % (self.first_mode, first_mode_param)

    return (s.getvalue(),tr)

  # Not implemented, doesn't have controller or data
  def BacklashPlot(self):
    s,tr = self.BacklashAnalysis()

    #create the figure
    fig=plot.figure(1)
    axes1 = fig.add_subplot(211)
    axes2 = fig.add_subplot(212)
    #axes3 = fig.add_subplot(313)
    axes1.set_xlabel('Effort')
    axes1.set_ylabel('Position')
    axes2.set_xlabel('Time')
    axes2.set_ylabel('Position')
    #plot the effort hysteresis
    axes1.plot(numpy.array(self.data.effort), numpy.array(self.data.position), 'r')
    #show the average effort lines 
    #axes1.axhline(y=self.data.arg_value[1],color='b')
    #axes1.axhline(y=0,color='k')
    #axes1.axhline(y=self.data.arg_value[0],color='b')
    #show that a constant velocity was achieved
    axes2.plot(numpy.array(self.data.position), 'b')
    #axes3.plot(numpy.array(self.data.effort), 'g')
    #axes3.plot(numpy.array(self.data.velocity),numpy.array(self.data.position), 'r')
    #pass along results
    result_service = rospy.ServiceProxy('test_result', TestResult)
    r = TestResultRequest()
    r.text_result = ""
    r.plots = []
    if tr==True:
      r.result =TestResultRequest.RESULT_PASS
    else:
      r.result = TestResultRequest.RESULT_HUMAN_REQUIRED
    
    stream = StringIO()
    plot.savefig(stream, format="png")
    image = stream.getvalue()
    
    p = qualification.msg.Plot()
    r.plots.append(p)
    p.text = s
    p.title = self.data.joint_name + "_backlash"
    p.image = image
    p.image_format = "png"
    result_service.call(r)


  def BacklashAnalysis(self):
    #compute the encoder travel
    s = StringIO()
    print >> s, "look at the data"
    tr=False
    return (s.getvalue(),tr)
    
if __name__ == "__main__":
  try:
    app = App()
    rospy.spin()
  except Exception, e:
    print e
    
  print 'quit'
