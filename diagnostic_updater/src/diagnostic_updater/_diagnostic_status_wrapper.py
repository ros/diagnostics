# -*- coding: utf-8 -*-

""" diagnostic_updater for Python.
@author Brice Rebsamen <brice [dot] rebsamen [gmail]>
"""

import roslib
roslib.load_manifest('diagnostic_updater')
import rospy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue



class DiagnosticStatusWrapper(DiagnosticStatus):
    """ Wrapper for the diagnostic_msgs::DiagnosticStatus message that makes it
    easier to update.

    This class handles common string formatting and vector handling issues
    for filling the diagnostic_msgs::DiagnosticStatus message. It is a subclass of
    diagnostic_msgs::DiagnosticStatus, so it can be passed directly to
    diagnostic publish calls.
    """

    def __init__(self, *args, **kwds):
        """
        Constructor. Any message fields that are implicitly/explicitly
        set to None will be assigned a default value. The recommend
        use is keyword arguments as this is more robust to future message
        changes.  You cannot mix in-order arguments and keyword arguments.

        The available fields are:
        level,name,message,hardware_id,values

        @param args: complete set of field values, in .msg order
        @param kwds: use keyword arguments corresponding to message field names
        to set specific fields.
        """
        DiagnosticStatus.__init__(self, *args, **kwds)


    def summary(self, *args):
        """ Fills out the level and message fields of the DiagnosticStatus.

        Usage:
        summary(diagnostic_status): Copies the summary from a DiagnosticStatus message
        summary(lvl,msg): sets from lvl and messages
        """
        if len(args)==1:
            self.level = args[0].level
            self.message = args[0].message
        elif len(args)==2:
            self.level = args[0]
            self.message = str(args[1])


    def clearSummary(self):
        """ Clears the summary, setting the level to zero and the message to "".
        """
        self.summary(0, "")


    def mergeSummary(self, *args):
        """ Merges a level and message with the existing ones.

        It is sometimes useful to merge two DiagnosticStatus messages. In that case,
        the key value pairs can be unioned, but the level and summary message
        have to be merged more intelligently. This function does the merge in
        an intelligent manner, combining the summary in *this, with the one
        that is passed in.

        The combined level is the greater of the two levels to be merged.
        If both levels are non-zero (not OK), the messages are combined with a
        semicolon separator. If only one level is zero, and the other is
        non-zero, the message for the zero level is discarded. If both are
        zero, the new message is ignored.

        Usage:
        mergeSummary(diagnostic_status): merge from a DiagnosticStatus message
        mergeSummary(lvl,msg): sets from lvl and msg
        """
        if len(args)==1:
            lvl = args[0].level
            msg = args[0].message
        elif len(args)==2:
            lvl = args[0]
            msg = args[1]

        if (lvl>0) == (self.level>0):
            if len(self.message)>0:
                self.message += "; "
            self.message += msg
        elif lvl > self.level:
            self.message = msg

        if lvl > self.level:
            self.level = lvl


    def add(self, key, val):
        """ Add a key-value pair.

        This method adds a key-value pair. Any type that has a << stream
        operator can be passed as the second argument.  Formatting is done
        using a std::stringstream.

        @type key string
        @param key Key to be added.
        @type value string
        @param value Value to be added.
        """
        self.values.append(KeyValue(key,str(val)))
