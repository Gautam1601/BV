#!/usr/bin/env python

import rospy
import actionlib
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal
from std_msgs.msg import String
from std_msgs.msg import Empty
from sensor_msgs.msg import Temperature 
from Movement_msgs.msg import StirringAction, PouringAction 

class BournvitaMaker:
    def __init__(self):
        rospy.init_node('bournvita_maker')

        # Init MoveIt
        self.moveit_client = actionlib.SimpleActionClient('/move_group', MoveGroupAction)
        self.moveit_client.wait_for_server()

        # Init temperature sensor 
        self.temperature_sub = rospy.Subscriber('/temperature', Temperature, self.temperature_callback)
        self.heating_pub = rospy.Publisher('/heat_control', Empty, queue_size=10)


        # Init ActionLib
        self.stirring_client = actionlib.SimpleActionClient('/stirring', StirringAction)
        self.stirring_client.wait_for_server()

        self.pouring_client = actionlib.SimpleActionClient('/pouring', PouringAction)
        self.pouring_client.wait_for_server()

    def activate_heating_and_move_pan(self):
    if pan_in_position:
        rospy.loginfo("Moving the pan to the heat source!")
        
        goal = MoveGroupGoal()
        goal.request = "plan_and_execute"
        goal.group_name = "pan_group"
        
        # Final position of the Pan 
        goal.pose_goal.position.x = 0.0
        goal.pose_goal.position.y = 0.0
        goal.pose_goal.position.z = 0.0
        goal.pose_goal.orientation.x = 0.0
        goal.pose_goal.orientation.y = 0.0
        goal.pose_goal.orientation.z = 0.0
        goal.pose_goal.orientation.w = 1.0

        # Switch on the Heating device once the Pan has been placed
        
        rospy.loginfo("Switching on the heating device")
        self.heating_pub.publish(Empty())
    
    else:
         rospy.logerr("Error: Pan is not on the Heating device or is not placed correctly!")

        
        pass

    def temperature_callback(self, temperature_msg):
              
        # Callback function for the temperature sensor
        current_temperature = temperature_msg.Temperature
        # Temp Sensor Activation to check if milk hot 
        desired_temperature = 40.0 # Can be adjusted 
        # Error Case identification
        if current_temperature >= desired_temperature:
            rospy.loginfo("Hot enough. Proceeding to the next step!")
            self.activate_bournvita_dispenser()  # If things go well
        else:
            rospy.loginfo("Not hot enough. Continue Heating!")

        pass

    def activate_bournvita_dispenser(self):
        # BV dispenser activation using AL
        pass

    def activate_stirring_module(self):
        # Activate the stirring module using AL
        # Continue stirring until BV dissolved
        stirring_goal = StirringGoal()  # Define  Stirring Goad
        self.stirring_client.send_goal(stirring_goal)
        self.stirring_client.wait_for_result()

#    Optional parts 
   
    # def activate_sugar_dispenser(self):
    #     # Activate the sugar dispenser using ActionLib
    #     # Implementation depends on your specific robot and environment
    #     pass

    # def taste_check_module(self):
    #     # Manual step, can be ignored in this automation script
    #     pass
    
# With human influence ^

    def move_pan_to_pouring_position(self):
        # Move Pan to pouring pos
        pass

    def activate_pouring_module(self):
        # Activate the pouring module using AL
        pouring_goal = PouringGoal()  # Define pouring goal
        self.pouring_client.send_goal(pouring_goal)
        self.pouring_client.wait_for_result()

if __name__ == '__main__':
    try:
        bournvita_maker = BournvitaMaker()
        bournvita_maker.activate_heating_and_move_pan()
        bournvita_maker.temperature_callback()  # Implement logic to check temperature
        bournvita_maker.activate_bournvita_dispenser()
        bournvita_maker.activate_stirring_module()
        
        # bournvita_maker.activate_sugar_dispenser()
        # bournvita_maker.taste_check_module()  # Ignoring for automation
        
        bournvita_maker.move_pan_to_pouring_position()
        bournvita_maker.activate_pouring_module()
    except rospy.ROSInterruptException:
        pass
