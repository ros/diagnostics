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

# Author: Kevin Watts

import roslib
roslib.load_manifest('qualification')

import sys, os, time, string
from xml.dom import minidom

from qualification.msg import *
from srv import *
from test import *
from result import *

from datetime import datetime
from time import strftime
from PIL import Image
from cStringIO import StringIO
import wx

import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText


RESULTS_DIR = os.path.join(roslib.packages.get_pkg_dir('qualification'), 'results')
TEMP_DIR = os.path.join(roslib.packages.get_pkg_dir('qualification'), 'results/temp/')

class SubTestResult:
    def __init__(self, test_name, msg):
        self._name = test_name
        self._result = False
        if msg.result == TestResultRequest.RESULT_PASS:
            self._result = True

        self._text_result = msg.html_result
        if msg.text_summary is not None:
            self._summary = msg.text_summary
        else:
            self._summary = ''

        self._msg = msg
        self._failure_txt = ''
        
        self._plots = []
        if msg.plots is not None:
            for plt in msg.plots:
                self._plots.append(plt)

        self.write_images(TEMP_DIR)

    def set_passfail(self, result):
        self._result = result
            
    def get_name(self):
        return self._name

    def get_passfail(self):
        return self._result

    def set_failure_text(self, txt):
        self._failure_txt = txt

    def filename(self):
        return self._name.replace(' ', '_').replace('/', '__') + '.html'

    def write_images(self, path):
        for plot in self._plots:
            stream = StringIO(plot.image)
            im  = Image.open(stream)
            
            img_file = path + plot.title + '.' + plot.image_format
            im.save(img_file)

    def html_image_result(self, img_path = TEMP_DIR):
        html = '<H5 ALIGN=CENTER>Result Details</H5>'
        
        # Put '<img src=\"IMG_PATH/%s.png\" /> % image_title' in html_result
        html += self._text_result.replace('IMG_PATH/', img_path)

        return html

    # Takes test title, status, summary, details and images and makes a 
    # readable and complete results page.
    # Can add a link back to the test index, or next subtest result
    # 
    # TODO: Make failure appear in red text
    # 
    def make_result_page(self, back_link = False, back_path = TEMP_DIR):
        html = "<html><head><title>Qualification Test Results: %s</title>\
<style type=\"text/css\">\
body { color: black; background: white; }\
div.warn { background: red; padding: 0.5em; border: none; }\
div.pass { background: green; padding: 0.5em; border: none; }\
strong { font-weight: bold; color: red; }\
em { font-style:normal; font-weight: bold; }\
</style>\
</head><body>\n" % self._name

        html += self.html_header()
        
        html += '<hr size="3">\n'
        
        # Parses through text and adds image source to file
        html += self.html_image_result()
        
        # Add link back to index if applicable
        # May want to make page "portable" so whole folder can move
        if back_link:
            back_index_file = back_path + 'index.html'
            html += '<p><a href="%s">Back to Index</a></p>' % back_index_file

        html += '</body></html>'
        
        return html 

    # The header of a subtest result, also called when diplaying 
    # short version of tests in index.html file
    def html_header(self):
        html = '<H4 ALIGN=CENTER>Results of %s</H4>\n' % self._name
        
        status_html = '<p>Status: <strong>FAIL</strong></p>\n'
        if (self._msg.result == TestResultRequest.RESULT_PASS):
            status_html = '<p>Status: <em>OK</em></p>\n'
        html += status_html

        if self._summary != '':
            html += '<p><em>Summary</em></p>\n' 
            html += '<p>%s</p>\n' % self._summary
        if self._failure_txt != '':
            html += '<p><em>Failure Reason</em></p>\n' 
            html += '<p>%s</p>\n' % self._failure_txt

        return html


class QualTestResult:
    def __init__(self, serial, start_time):
        self._subresults = {}
        self._subresults_by_index = {}
        self._start_time = start_time
        self._start_time_filestr = self._start_time.strftime("%Y%m%d_%H%M")
        self._start_time_name = self._start_time.strftime("%Y/%m/%d %I:%M%p")
        self._serial = serial

        self._results_dir = os.path.join(RESULTS_DIR, '%s_%s/' % (self._serial, self._start_time_filestr))
        if not os.path.isdir(self._results_dir):
            self._made_dir = self._results_dir
            os.mkdir(self._results_dir)
        else:
            self._made_dir = ''

        # Data values used when reconfiguring a part
        self.config_only = False
        self._conf_name = ''

        self.has_error_no_invent = False
        self.canceled = False
        self.dev_team = [ "watts@willowgarage.com" ]

        self._note = ''
        self._operator = ''

    def __del__(self):
        # Delete extra directories if empty
        if self._made_dir != '' and len(os.listdir(self._made_dir)) == 0:
            print 'Removing directory %s' % self._made_dir
            os.rmdir(self._made_dir)
       

    def set_notes(self, note):
        self._note = note

    def set_operator(self, name):
        self._operator = name

    def get_subresults(self):
        kys = dict.keys(self._subresults)
        kys.sort()
        vals = []
        for ky in kys:
            vals.append(self._subresults[ky])
        
        # return sorted by index
        return vals
        
    def add_sub_result(self, index, test_name, msg):
        sub = SubTestResult(test_name, msg)
        
        self._subresults[test_name] = sub
        self._subresults_by_index[index] = sub
        return sub

    # Come up with better enforcement of get functions
    def get_subresult(self, index):
        return self._subresults_by_index[index]

    def test_result(self):
        if len(self.get_subresults()) == 0:
            return False

        if self.canceled:
            return False

        for res in self.get_subresults():
            if not res.get_passfail():
                return False

        return True

    def test_status_str(self):
        result = self.test_result()
        if result:
            return 'PASS'
        else:
            return 'FAIL'

    def make_log_table(self):
        if self._test_log is None or self._test_log == '':
            return ''

        # Sort test log by times
        kys = dict.keys(self._test_log)
        kys.sort()
        
        html = '<H4 AlIGN=CENTER>Test Log Data</H4>\n'
        html += '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td><b>Time</b></td><td><b>Log Message</b></td></tr></b>\n'
        for ky in kys:
            time_str = ky.strftime("%m/%d/%Y %H:%M:%S")
            html += '<tr><td>%s</td><td>%s</td></tr>\n' % (time_str, self._test_log[ky])
        html += '</table>'

        return html
        

    def make_summary_page(self, link = True, write_dir = TEMP_DIR):
        html = "<html><head>\n"
        html += "<title>Qualification Test Result Summary for %s: %s</title>\n" % (self._serial, self._start_time_name)
        html += "<style type=\"text/css\">\
body { color: black; background: white; }\
div.warn { background: red; padding: 0.5em; border: none; }\
div.pass { background: green; padding: 0.5em; border: none; }\
strong { font-weight: bold; color: red; }\
em { font-style: normal; font-weight: bold; }\
</style>\
</head>\n<body>\n"

        if not self.config_only:
            html += '<H2 ALIGN=CENTER>Qualification of: %s</H2>\n<br>\n' % self._serial
        else:
            html += '<H2 ALIGN=CENTER>Configuration of: %s</H2>\n<br>\n' % self._serial

        result = '<H3>Test Result: <strong>FAIL</strong></H3>\n' 
        if self.test_result():
            result = '<H3>Test Result: <em>PASS</em></H3>\n'
        
        html += result
        html += '<HR size="2">'

        if self._operator == '':
            operator = 'Unknown'
        else:
            operator = self._operator

        if self._note is None or self._note == '':
            self._note = 'No notes given.'
        
        html += '<H5><b>Test Engineer\'s Notes</b></H5>\n'
        html += '<p><b>Test Engineer: %s</b></p>\n' % operator        
        html += '<p>%s</p>\n' % self._note

        html += '<H5>Test Date</H5>\n'
        html += '<p>Completed Test at %s on %s.</p>\n' % (self._start_time_name, self._serial)
        html += '<H5><b>Results Directory</b></H5>\n<p>%s</p>\n' % self._results_dir

        if self.canceled:
            html += '<p><b>Test canceled by operator.</b></p>\n'

        if len(self.get_subresults()) == 0 and not self.has_error_no_invent:
            html += '<p>No subtests completed. Test may have ended badly.</p>\n'
        elif len(self.get_subresults()) == 0 and self.has_error_no_invent:
            html += '<p>Error during pretests. Check system and retry.</p>\n'
        else:
            html += '<HR size="2">'
            # Index items link to each page if link 
            html += self.make_index(link, write_dir)
            
        html += '<hr size="4">\n'
        html += self.make_log_table()
        html += '<hr size="4">\n'

        
        html += '</body></html>'

        return html

    def line_summary(self):
        result = "FAIL"
        if self.test_result():
            result = "PASS"

        if self.config_only and self.test_result():
            sum = "Reconfigured of %s as %s." % (self._serial, self._conf_name)
        else:
            sum = "Qualification of %s, Result: %s" % (self._serial, result)            
        if self.has_error_no_invent:
            sum += ". Test ERROR"
        if self.canceled:
            sum += ". Test CANCEL"
        return sum

    def make_index(self, link, folder):
        html = '<H4 AlIGN=CENTER>Results Index</H4>\n'
        html += '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td><b>Test Name</b></td><td><b>Summary</b></td><td><b>Final Result</b></td></tr></b>\n'

        for st in self.get_subresults():
            html += self.make_subresult_index_line(st, link, folder)

        html += '</table>\n'
        
        return html

    # Moved to subresult class?
    def make_subresult_index_line(self, subresult, link, folder):
        result = '<div class="warn">FAIL</div>'
        if subresult.get_passfail():
            result = '<div class="pass">OK</div>'
        
        path = folder + subresult.filename()
        if link:
            hyperlink = '<a href=\"%s\">%s</a>' % (path, subresult._name)
        else:
            hyperlink = subresult._name

        summary = subresult._summary + '\n' + subresult._failure_txt

        return '<tr><td>%s</td><td>%s</td><td>%s</td></tr>\n' % (hyperlink, summary, result)
        
    def write_results_to_file(self, temp = True):
        # Write into temp or final dir
        # Temp for display in Qual GUI, final for storing for later
        write_dir = TEMP_DIR
        if not temp:
            write_dir = self._results_dir

        if (not os.path.isdir(write_dir)):
            os.mkdir(write_dir)
              
        index_path = write_dir + 'index.html'
        index = open(index_path, 'w')
        index.write(self.make_summary_page(True, write_dir))
        index.close()
        
        for st in self.get_subresults():
            st_path = write_dir + st.filename()
            st_file = open(st_path, 'w')
            # Set back path for backlinks here, if needed
            st_file.write(st.make_result_page(True, write_dir))
            st_file.close()

            st.write_images(write_dir)

    # Make invent results pretty, HTML links work
    def log_results_invent(self, invent):
        if invent == None or self.has_error_no_invent:
            return
        if len(self.get_subresults()) == 0:
            return
        
        prefix = self._start_time_filestr + "_" # Put prefix first so images sorted by date

        if self.config_only:
            sub = self.get_subresult(0)
            invent.setNote(self._serial, self.line_summary())
            invent.add_attachment(self._serial, prefix + sub.filename(), 'text/html', sub.make_result_page())
            return

        invent.setKV(self._serial, "Test Status", self.test_status_str())
        test_log = "Test run on %s, status: %s."%(self._start_time_name, self.test_status_str())
        invent.setNote(self._serial, test_log)
        
        
        invent.add_attachment(self._serial, prefix + "summary.html", "text/html", self.make_summary_page(False))
        
        for sub in self.get_subresults():
            invent.add_attachment(self._serial, prefix + sub.filename(), 'text/html', sub.make_result_page())
            for plt in sub._plots:
                invent.add_attachment(self._serial, prefix + plt.title + '.' + plt.image_format, "image/" + plt.image_format, plt.image)
            

    def get_dev_team(self):
        return string.join(self.dev_team, ", ")

    def email_dev_team(self):
        try:
            if self._operator is not None and self._operator != '':
                sender = "%s@willowgarage.com" % self._operator
            else:
                sender = "watts@willowgarage.com"

            msg = MIMEMultipart('alternative')
            msg['Subject'] = "--QualResult-- %s" % self.line_summary()
            msg['From'] = sender
            msg['To'] = self.get_dev_team()

            msg.attach(MIMEText(self.make_summary_page(False), 'html'))

            s = smtplib.SMTP('localhost')
            s.sendmail(sender, self.get_dev_team(), msg.as_string())
            s.quit()
        except Exception, e:
            print 'Unable to sent mail, caught exception!'
            print e
