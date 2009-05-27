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

# Author: Kevin Watts

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
import matplotlib.pyplot as plot
from StringIO import StringIO

from qualification.msg import Plot
from qualification.srv import *

from joint_qualification_controllers.srv import *

import traceback

class App:
  def __init__(self):
    rospy.init_node("TestPlotter", anonymous=True)
    self.data_topic = rospy.Service("/test_data", TestData, self.on_data)
    self.result_service = rospy.ServiceProxy('test_result', TestResult)
    rospy.spin()
    
  def on_data(self,req):
    print 'Got data named %s' % (req.test_name)
    self.data = req
    if self.data.test_name == "hysteresis":
      self.hysteresis_plot()
    elif self.data.test_name == "sinesweep":
      self.sine_sweep_plot()
    else:
      rospy.logerr('Recieved test message with name %s, unable to analyze' % s)
      self.test_failed_service_call('Unable to analyze result.')
    return TestDataResponse(1)
      
  def test_failed_service_call(self, except_str = ''):
    rospy.logerr(except_str)
    r = TestResultRequest()
    r.html_result = except_str
    r.text_summary = 'Caught exception, automated test failure.'
    r.plots = []
    r.result = TestResultRequest.RESULT_FAIL
    self.result_service.call(r)

  def hysteresis_plot(self):
    try:
      image_title = self.data.joint_name + "_hysteresis"

      text, summary, tr = self.hysteresis_analysis(image_title)
      
      # Create the figure 1: Effort vs. Position. 2: Velocity vs. Position
      fig=plot.figure(1)
      axes1 = fig.add_subplot(211)
      axes2 = fig.add_subplot(212)
      axes1.set_xlabel('Position')
      axes1.set_ylabel('Effort')
      axes2.set_xlabel('Position')
      axes2.set_ylabel('Velocity')
      
      # Plot the effort hysteresis
      axes1.plot(numpy.array(self.data.position), numpy.array(self.data.effort), 'r--')
      # Show the expected average effort lines in blue
      axes1.axhline(y = self.data.arg_value[1], color = 'b')
      axes1.axhline(y = 0, color = 'k')
      axes1.axhline(y = self.data.arg_value[0], color = 'b')
      # Show actual average effort lines in green
      if self.max_avg != 0 or self.min_avg != 0:
        axes1.axhline(y = self.max_avg, color='g')
        axes1.axhline(y = self.min_avg, color='g')

      # Show that a constant velocity was achieved, plot expected vel in green
      axes2.plot(numpy.array(self.data.position), numpy.array(self.data.velocity), 'b--')
      axes2.axhline(y = self.data.arg_value[4], color = 'g')
      axes2.axhline(y = -1 * self.data.arg_value[4], color = 'g')
      
      # Joint name on title
      fig.text(.35, .95, self.data.joint_name + ' Hysteresis Test')
      
      # Pass along results to qual GUI
      r = TestResultRequest()
      r.html_result = text
      r.text_summary = summary

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
      p.title = image_title
      p.image = image
      p.image_format = "png"
      self.result_service.call(r)
    except Exception, e:
      rospy.logerr('hysteresis_plot caught exception, returning test failure.')
      rospy.logerr(traceback.format_exc())
      self.test_failed_service_call(traceback.format_exc())

  def hysteresis_analysis(self, image_title):
    self.max_avg = 0
    self.min_avg = 0

    # Check data array for mininum number of points
    num_pts = numpy.array(self.data.position).size

    print "Num pts %s." % num_pts

    if num_pts < 250: 
      error_msg = "<p>Not enough data points, hysteresis controller may have malfunctioned. Check diagnostics.</p>"
      error_msg += "<p>The encoder may be missing ticks, check encoder.<br>\n"
      error_msg += "Make sure mechanism is free to move and not trapped or constrained<br>\n"
      error_msg += "Make sure motors are not in safety lockout (check diagnostics)<br>\n"
      error_msg += "Check controller gains if problem persists on components of same type.</p>\n"
      error_msg += "<p>Test status: <p>FAIL</b>.</p>"
      print error_msg
      return (error_msg, "Not enough data points, bad encoder or bad gains.", False)

    # Expected values
    min_effort_param = self.data.arg_value[0]
    max_effort_param = self.data.arg_value[1]
    min_range_param  = self.data.arg_value[2]
    max_range_param  = self.data.arg_value[3]
    
    # Compute the encoder travel
    min_encoder = min(numpy.array(self.data.position))
    max_encoder = max(numpy.array(self.data.position))

    if abs(max_encoder - min_encoder) < 0.001:
      error_msg = "<p>No travel of mechanism, hysteresis did not complete. Check controller gains and encoder.</p><p>Test status: <b>FAIL</b>.</p>"
      print error_msg
      return (error_msg, "No travel in mechanism. Bad encoder, bad gains, or motors on lockout.", False)

    # Find the index to do the average over
    # Data set starts close to one end of hysteresis
    # Find endpoint of travel closest to midpoint of data
    index_max = numpy.argmax(numpy.array(self.data.position))
    index_min = numpy.argmin(numpy.array(self.data.position))
    end = numpy.array(self.data.position).size
    if (abs(end/2 - index_max) < abs(end/2 - index_min)):
      index = index_max
    else:
      index = index_min

    # Find indices to compute averages / SD over
    # Take middle 80% of array by index
    effort1_array = numpy.array(self.data.effort)[int(0.1 * index): int(0.9 * index)]
    effort2_array = numpy.array(self.data.effort)[int(1.1 * index): max(int(1.9 * index), end - int(0.1 * index))]

    # Make sure we have at least some points in both directions
    if effort1_array.size < 20 or effort2_array.size < 20:
      error_msg = '<p>Not enough data in one or more directions.</p><p>Test Status: <b>FAIL</b>.</p>'
      print error_msg
      return (error_msg, 'No data in one or more directions.', False)

    #compute the average efforts with std deviations to display
    effort1 = numpy.average(effort1_array)
    effort2 = numpy.average(effort2_array)

    effort1_sd = numpy.std(effort1_array)
    effort2_sd = numpy.std(effort2_array)

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
    
    tr = True
    range_msg    = "OK"
    negative_msg = "OK"
    positive_msg = "OK"
    error_html = ""
    summary_txt = ""
    
    # Check to range against expected values
    if (min_encoder > min_range_param and max_encoder < max_range_param) and (min_range_param != 0 and max_range_param != 0):
      error_html += "Mechanism is not traveling the complete distance!<br>"
      range_msg = "FAIL"
      summary_txt += "Range: FAIL. "
      tr = False
    else:
      summary_txt += "Range: OK. "

    # Check effort limits (within 15% of expected)
    if abs((self.min_avg - min_effort_param)/min_effort_param) > 0.15:
      error_html += "<br>The mechanism average effort in the negative direction is out of bounds!<br>"
      negative_msg = "FAIL"
      tr = False
    
    summary_txt += "Negative Effort: %s" % negative_msg

    # Check that effort is even (<20% standard deviation)
    if abs(self.min_sd / self.min_avg) > 0.20:
      error_html += "Effort is uneven in negative direction.<br>"
      negative_msg += " absolute value, uneven effort!"
      tr = False
      summary_txt += ", uneven. "
    else:
      summary_txt += ". "

    # Positive efforts
    if abs((self.max_avg - max_effort_param)/max_effort_param) > 0.15:
      error_html += "The mechanism average effort in the positive direction is out of bounds!<br>"
      positive_msg = "FAIL"
      tr = False
    
    summary_txt += "Positive Effort: %s" % positive_msg

    if abs(self.max_sd / self.max_avg) > 0.20:
      error_html += "Effort is uneven in positive direction."
      positive_msg += " absolute value, uneven effort!"
      tr = False
      summary_txt += ", uneven. "
    else:
      summary_txt += ". "

    if tr:
      test_msg = "FAIL"
    else:
      test_msg = "PASS"

    # Standard deviations in percent of value
    sd_min_percent = abs(self.min_sd / self.min_avg) * 100
    sd_max_percent = abs(self.max_sd / self.max_avg) * 100

    html_result = '<H6>Efforts</H6>'
    html_result += "<p>Positive. Average: %f, SD: %f percent. Expected: %f. Status: %s.</p>" % (self.max_avg, sd_max_percent, max_effort_param, positive_msg)  
    html_result += "<p>Negative. Average: %f, SD: %f percent. Expected: %f. Status: %s.</p>" % (self.min_avg, sd_min_percent, min_effort_param, negative_msg)  
    html_result += "<H6>Joint limits</H6>"
    html_result += "<p>Max measured: %f, expected: %f. Min measured: %f, expected: %f: Status: %s.</p>" % (max_encoder, max_range_param, min_encoder, min_range_param, range_msg)

    html_result += '<br><img src=\"IMG_PATH/%s.png\", width = 640, height = 480/><br>' % image_title

    
    if not tr:
      html_result += "<H6>Test failure data</H6>"
      html_result += "<p>%s</p>" % error_html

    return (html_result, summary_txt, tr)

  def sine_sweep_plot(self):
    try:

      r = TestResultRequest()
      r.html_result = ''
      r.text_summary = 'No data.'
      r.plots = []
      r.result = TestResultRequest.RESULT_FAIL

      image_title = self.data.joint_name + "_sine_sweep"

      # Compute the encoder travel
      min_encoder = min(numpy.array(self.data.position))
      max_encoder = max(numpy.array(self.data.position))
      
      if abs(max_encoder - min_encoder) < 0.001:
        r.html_result = "<p>No travel of mechanism, sinesweep did not complete. Check controller gains and encoder.</p><p>Test status: <b>FAIL</b>.</p>"
        r.text_summary = "No travel of mechanism. Bad encoder, bad gains, or motors on lockout."
        self.result_service.call(r)
      
      # Plot the values and line of best fit
      fig = plot.figure(1)
      axes1 = fig.add_subplot(211)
      axes1.clear()
      axes2 = fig.add_subplot(212)
      axes2.clear()
      # Find the next power of two above our number of data points
      next_pow_two=int(2**math.ceil(math.log(numpy.array(self.data.position).size,2)))
      # plot in decibels
      axes1.psd(numpy.array(self.data.effort), NFFT=next_pow_two, Fs=1000, Fc=0, color='r')
      axes1.psd(numpy.array(self.data.position), NFFT=next_pow_two, Fs=1000, Fc=0)
      axes1.set_xlim(0, 100)
      axes1.set_title('Position PSD')
    
      # plot in power (pxx - power, f - freqs)
      pxx, f = axes2.psd(numpy.array(self.data.velocity), NFFT=next_pow_two, Fs=1000, Fc=0)
      axes2.clear()
      for i in f:
        if f[i]>4:
          cutoff=i # Set cutoff to avoid finding first mode at extremely low freqs
          break
        axes2.plot(f,pxx)

      # find the peak
      index = numpy.argmax(pxx[cutoff:pxx.size])
      index = index + cutoff
      max_value = max(pxx[cutoff:pxx.size])
      axes2.plot([f[index]],[pxx[index]],'r.', markersize = 10);
      self.first_mode = f[index]

      # Find second mode
      # Max after first mode
      index2 = numpy.argmax(pxx[index - cutoff: pxx.size])
      max_value = max(pxx[index - cutoff: pxx.size])
      index2 += cutoff
      axes2.plot([f[index2]], [pxx[index2]], 'r.', markersize = 8);
      self.second_mode = f[index2]
      
      r.html_result, r.text_summary, tr = self.sine_sweep_analysis(image_title)
      axes2.axvline(x=self.data.arg_value[0], color='r') # Line at first mode
      axes2.set_xlim(0, 100)
      axes2.set_ylim(0, max_value+10)
      axes2.set_xlabel('Frequency')
      axes2.set_ylabel('Power')
      axes2.set_title('Velocity PSD')
      
      # Title
      fig.text(.35, .95, self.data.joint_name + ' SineSweep Test')

      
      #r.html_result = html
      #r.text_summary = summary
      if tr==True:
        r.result = TestResultRequest.RESULT_PASS
      else:
        r.result = TestResultRequest.RESULT_HUMAN_REQUIRED
    
      stream = StringIO()
      plot.savefig(stream, format="png")
      image = stream.getvalue()
      
      
      p = qualification.msg.Plot()
      r.plots.append(p)
      p.title = image_title
      p.image = image
      p.image_format = "png"
      self.result_service.call(r)
    except Exception, e:
      rospy.logerr('sine_sweep_plot caught exception, returning test failure.')
      rospy.logerr(traceback.format_exc())
      self.test_failed_service_call(traceback.format_exc())

  def sine_sweep_analysis(self, image_title): 
    # Check data array for mininum number of points
    num_pts = numpy.array(self.data.position).size

    if num_pts < 101: 
      tr = False
      error_msg = "<p>Not enough data points in position array. Minimum 101, found %d. Check encoder and amplitude of controller.</p>" % num_pts
      return (error_msg, "Not enough data points.", tr)
  
    # Parameters
    first_mode_param = self.data.arg_value[0]
    second_mode_param = self.data.arg_value[1]
    mode_error_param = self.data.arg_value[2]

    html = ''
    summary = ''
    if abs(self.first_mode - first_mode_param)/first_mode_param > mode_error_param:
      html += "<H6>The first mode is incorrect: <b>FAIL</b>.</H6>"
      summary += "First mode: FAIL."
      tr=False
    else:
      html += "<H6>First mode: <b>OK</b>.</H6>"
      summary += "First mode: OK."
      tr=True
    html += "<p>First mode measured: %f, expected: %f.</p>" % (self.first_mode, first_mode_param)
    #html += "<p>Second mode measured: %f, expected %f.</p>" % (self.second_mode, second_mode_param)

    html += '<br><img src=\"IMG_PATH/%s.png\", width = 640, height = 480/><br>' % image_title

    return (html, summary, tr)

     
if __name__ == "__main__":
  try:
    app = App()
    rospy.spin()
  except Exception, e:
    print e
    
  print 'Quitting Hysteresis Sinesweep Plot, shutting down node.'
