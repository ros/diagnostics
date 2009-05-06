#!/usr/bin/env python



import roslib
roslib.load_manifest('qualification')

import rospy
from qualification.srv import * 


rospy.wait_for_service('shutdown_done', 3)
done_proxy = rospy.ServiceProxy('shutdown_done', ScriptDone)
done  = ScriptDoneRequest()
done.result = ScriptDoneRequest.RESULT_OK
done.script = 'shutdown.launch'
done.failure_msg = ''
done_proxy.call(done)
