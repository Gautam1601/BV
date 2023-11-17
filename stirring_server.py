#!/usr/bin/env python

import rospy
import actionlib
from Movement_msgs.msg import StirringAction, StirringResult

class StirringServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('/stirring', StirringAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        # ...

        # Once the stirring is complete, Finish
        result = StirringResult()
        result.success = True
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('stirring_server')
    stirring_server = StirringServer()
    rospy.spin()

