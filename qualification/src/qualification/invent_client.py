#! /usr/bin/env python

"""
usage: %(progname)s --username=... --password=... reference key value

  * sets a key/value oo an item (referenced by 'reference')
"""


import os, sys, string, time, getopt

import urllib2, cookielib

import mimetypes
import mimetools

class Invent:
  def __init__(self, username, password):
    self.username = username
    self.password = password

    self.cj = cookielib.CookieJar()
    self.opener = urllib2.build_opener(urllib2.HTTPCookieProcessor(self.cj))

    self.loggedin = False

    self.site = "http://invent.willowgarage.com/invent/"
    #self.site = "http://localhost/sinvent/"

  def login(self):
    username = self.username
    password = self.password
    url = self.site + "login/signin0.py?Action.Login=1&username=%(username)s&password=%(password)s" % locals()

    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

    self.loggedin = False
    if body.find("Invalid Login") != -1:
      return False

    self.loggedin = True
    return True

  def setNote(self, reference, note):
    if self.loggedin == False:
      self.login()

    url = self.site + "invent/api.py?Action.AddNoteToItem=1&reference=%s&note=%s" % (reference, urllib2.quote(note))

    fp = self.opener.open(url)
    fp.read()
    fp.close()

  def setKV(self, reference, key, value):
    if self.loggedin == False:
      self.login()

    key = key.strip()
    value = value.strip()

    if not key:
      raise ValueError, "the key is blank"
    if not value:
      raise ValueError, "the value is blank"

    url = self.site + "invent/api.py?Action.setKeyValue=1&reference=%s&key=%s&value=%s" % (reference, urllib2.quote(key), urllib2.quote(value))

    fp = self.opener.open(url)
    fp.read()
    fp.close()

  def add_attachment(self, reference, name, mimetype, attachment):
    if self.loggedin == False:
      self.login()

    if not name:
      raise ValueError, "the name is blank"

    theURL = self.site + "invent/api.py"


    fields = []
    fields.append(('Action.addAttachment', "1"))
    fields.append(('reference', reference))
    fields.append(('mimetype', mimetype))
    fields.append(('name', name))

    files = []
    files.append(("attach", name, attachment))

    input = build_request(theURL, fields, files)

    response = self.opener.open(input).read()


## -------------------------------------------------------------

def build_request(theurl, fields, files, txheaders=None):
  content_type, body = encode_multipart_formdata(fields, files)
  if not txheaders: txheaders = {}
  txheaders['Content-type'] = content_type
  txheaders['Content-length'] = str(len(body))
  return urllib2.Request(theurl, body, txheaders)

def encode_multipart_formdata(fields, files, BOUNDARY = '-----'+mimetools.choose_boundary()+'-----'):

    """ Encodes fields and files for uploading.

    fields is a sequence of (name, value) elements for regular form fields - or a dictionary.

    files is a sequence of (name, filename, value) elements for data to be uploaded as files.

    Return (content_type, body) ready for urllib2.Request instance

    You can optionally pass in a boundary string to use or we'll let mimetools provide one.

    """    

    CRLF = '\r\n'

    L = []

    if isinstance(fields, dict):
        fields = fields.items()

    for (key, value) in fields:
        L.append('--' + BOUNDARY)
        L.append('Content-Disposition: form-data; name="%s"' % key)
        L.append('')
        L.append(value)

    for (key, filename, value) in files:
        filetype = mimetypes.guess_type(filename)[0] or 'application/octet-stream'
        L.append('--' + BOUNDARY)
        L.append('Content-Disposition: form-data; name="%s"; filename="%s"' % (key, filename))

        L.append('Content-Type: %s' % filetype)
        L.append('')
        L.append(value)

    L.append('--' + BOUNDARY + '--')
    L.append('')
    body = ''
    import array
    buff = array.array('c')
    for s in L:
      buff.fromstring(s)
      buff.fromstring(CRLF)
    body = buff.tostring()

    content_type = 'multipart/form-data; boundary=%s' % BOUNDARY        # XXX what if no files are encoded

    return content_type, body
