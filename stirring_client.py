#!/usr/bin/env python

import rospy
import actionlib
from Movement_msgs.msg import StirringAction, StirringGoal

class StirringClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/stirring', StirringAction)
        self.client.wait_for_server()

    def send_goal(self, duration=5.0, direction=StirringGoal.CLOCKWISE, speed=1.0):
        goal = StirringGoal()
        goal.duration = rospy.Duration(duration)
        goal.direction = direction
        goal.speed = speed

        rospy.loginfo(f"Send stirring goal with duration={duration}s, direction={direction}, speed={speed}")

        self.client.send_goal(goal)

        # Wait for the result
        if self.client.wait_for_result(rospy.Duration(30.0)):
            rospy.loginfo("Stirring completed successfully!")
        else:
            rospy.logwarn("Stirring did not complete within the specified time.")

if __name__ == '__main__':
    rospy.init_node('stirring_client')
    stirring_client = StirringClient()
    stirring_client.send_goal()

