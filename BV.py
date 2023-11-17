#!/usr/bin/env python

import rospy
import actionlib
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal
from std_msgs.msg import String
from std_msgs.msg import Empty
from sensor_msgs.msg import Temperature 
from Movement_msgs.msg import StirringAction, PouringAction, DispenserAction  

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
        else:
            rospy.loginfo("Not hot enough. Continue Heating!")
            temperature_callback()
        pass

    def activate_bournvita_dispenser(self, dispenser_parameter="default_value"):
        # Logic for Bournvita dispenser activation using AL with variable parameters

        rospy.loginfo("Activating the Bournvita dispenser!")

        bournvita_goal = DispenserAction()
        bournvita_goal.parameter = dispenser_parameter

        self.dispenser_client.send_goal(bournvita_goal)
        self.dispenser_client.wait_for_result()

        rospy.loginfo("Bournvita dispenser activated with parameter: {}".format(dispenser_parameter))
        pass

    def activate_stirring_module(self):
        # Activate the stirring module using AL
        # Continue stirring until BV dissolved
        stirring_goal = StirringGoal()  # Define  Stirring Goad
        self.stirring_client.send_goal(stirring_goal)
        self.stirring_client.wait_for_result()


    def move_pan_to_pouring_position(self, position_x=0.0, position_y=0.0, position_z=0.0,
                                  orientation_x=0.0, orientation_y=0.0, orientation_z=0.0, orientation_w=1.0):
        rospy.loginfo("Moving the pan to the pouring position!")

        goal = MoveGroupGoal()
        goal.request = "plan_and_execute"
        goal.group_name = "pan_group"

        # Final Pos
        goal.pose_goal.position.x = position_x
        goal.pose_goal.position.y = position_y
        goal.pose_goal.position.z = position_z
        goal.pose_goal.orientation.x = orientation_x
        goal.pose_goal.orientation.y = orientation_y
        goal.pose_goal.orientation.z = orientation_z
        goal.pose_goal.orientation.w = orientation_w
    
        self.moveit_client.send_goal(goal)
        self.moveit_client.wait_for_result()
    
        rospy.loginfo("Pan moved to the pouring position!")
        pass

    def activate_pouring_module(self, pouring_duration=5.0, pouring_speed=1.0):
        rospy.loginfo("Activating the pouring module!")
    
        pouring_goal = PouringAction()
        pouring_goal.duration = rospy.Duration(pouring_duration)
        pouring_goal.speed = pouring_speed
    
        self.pouring_client.send_goal(pouring_goal)
        self.pouring_client.wait_for_result()
    
        rospy.loginfo("Pouring module completed!")


if __name__ == '__main__':
    try:
        bournvita_maker = BournvitaMaker()
        bournvita_maker.activate_heating_and_move_pan()
        bournvita_maker.temperature_callback()
        bournvita_maker.activate_bournvita_dispenser()
        bournvita_maker.activate_stirring_module()
        bournvita_maker.move_pan_to_pouring_position()
        bournvita_maker.activate_pouring_module()
    except rospy.ROSInterruptException:
        pass
