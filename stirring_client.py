#!/usr/bin/env python

import rospy
import actionlib
from Movement_msgs.msg import StirringAction, StirringGoal

class StirringClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/stirring', StirringAction)
        self.client.wait_for_server()

    def send_goal(self):
        goal = StirringGoal()
        # Set your stirring goal parameters
        # ...

        self.client.send_goal(goal)
        self.client.wait_for_result()

if __name__ == '__main__':
    rospy.init_node('stirring_client')
    stirring_client = StirringClient()
    stirring_client.send_goal()

