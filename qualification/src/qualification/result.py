#!/usr/bin/env python

# Author: Kevin Watts

import roslib
roslib.load_manifest('qualification')

import sys, os, time
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

RESULTS_DIR = os.path.join(roslib.packages.get_pkg_dir('qualification'), 'results')
TEMP_DIR = os.path.join(roslib.packages.get_pkg_dir('qualification'), 'results/temp/')

class SubTestResult:
    def __init__(self, test_name, msg, path):
        self._name = test_name
        self._result = msg.result
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

        self._path = path
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
        
        
        self._note = ''
        self._operator = ''

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
        
        # return dict.values(self._subresults)
        return vals
        
    def add_sub_result(self, index, test_name, msg):
        sub = SubTestResult(test_name, msg, self._results_dir)
        
        self._subresults[test_name] = sub
        self._subresults_by_index[index] = sub
        return sub

    # Come up with better enforcement of get functions
    def get_subresult(self, index):
        return self._subresults_by_index[index]

    def test_result(self):
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

    def make_summary_page(self, link = True, write_dir = TEMP_DIR):
        html = "<html><head><title>Qualification Test Result Summary for %s: %s</title>\
<style type=\"text/css\">\
body { color: black; background: white; }\
div.warn { background: red; padding: 0.5em; border: none; }\
div.pass { background: green; padding: 0.5em; border: none; }\
strong { font-weight: bold; color: red; }\
em { font-style: normal; font-weight: bold; }\
</style>\
</head><body>\n" % (self._serial, self._start_time_name)

        html += '<H2 ALIGN=CENTER>Qualification of: %s</H2>\n' % self._serial
        #html += '<p ALIGN=CENTER>Test Completed on Date: %s</p>\n' % self._start_time_name
        
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
        
        html += '<H4><b>Test Engineer\'s Notes</b><H4>\n'
        html += '<p><b>Test Engineer: %s</b></p>\n' % operator        
        html += '<p>%s</p>\n' % self._note

        html += '<p>Completed Test at %s on %s.</p>' % (self._start_time_name, self._serial)
        html += '<HR size="2">'

        # Index items link to each page if link 
        html += self.make_index(link, write_dir)

        html += '<hr size="4">\n'
        #for st in self.get_subresults():
        #    html += st.html_header()
        #    html += '<hr size="3">\n'

        html += '</body></html>'

        return html

    def make_index(self, link, folder):
        html = '<H4 AlIGN=CENTER>Results Index</H4>\n'
        html += '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><b><td>Test Name</td><td>Summary</td><td>Final Result</td></b></tr>\n'

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

    # Verify that inventory storage still works for new log system
    def log_results_invent(self, invent):
        if invent == None:
            return

        invent.setKV(self._serial, "Test Status", self.test_status_str())
        test_log = "Test run on %s, status: %s."%(self._start_time_name, self.test_status_str())
        invent.setNote(self._serial, test_log)
        
        prefix = self._start_time_filestr + "_" # Put prefix first so images sorted by date
        invent.add_attachment(self._serial, prefix + "summary.html", "text/html", self.make_summary_page(False))
        
        for sub in self.get_subresults():
            invent.add_attachment(self._serial, prefix + sub.filename(), 'text/html', sub.make_result_page())
            for plt in sub._plots:
                invent.add_attachment(self._serial, prefix + plt.title + '.' + plt.image_format, "image/" + plt.image_format, plt.image)
            

    
        
