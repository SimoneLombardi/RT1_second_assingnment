#! /usr/bin/env python3
"""
.. module: lastGoal_srv 

.. moduleauthor:: Simone Lombardi

This node implements the server for the service lastGoal. This module returns the last goal position as a response to the service call.
The node listen on the topic /reaching_goal/goal, this topic is part of the action server topic list and when a new goal is set by the user the coordinates are broadcasted on this topic.
The node saves the last goal position in a global variable and when the service is called it returns the last goal position as a response.

Subscriber:
    /reaching_goal/goal: The goal position of the robot.

Publisher:
    None

Service:
    lastGoal: The service that returns the last goal position.
"""

import rospy
import time
import random

import assignment_2_2023.msg

from assignment_2_2023.srv import Last_goal, Last_goalResponse
from assignment_2_2023.msg import PlanningActionGoal

last_goal = assignment_2_2023.msg.PlanningActionGoal()

def srv_callback(request):
    """
    Callback function for the service lastGoal.

    Args:
        request (assignment_2_2023.srv.Last_goalRequest): The request of the service.

    Returns:
        assignment_2_2023.srv.Last_goalResponse: The response of the service.

    This callback function is used by the service lastGoal to return the last goal position as a response to the service call, using the value saved in the variable 
    last_goal.goal.target_pose.pose.position.x and last_goal.goal.target_pose.pose.position.y the function fills the response object and returns it.
    """
    # create a response object
    response = Last_goalResponse()
    # fill the response object with the last goal position
    response.x = last_goal.goal.target_pose.pose.position.x
    response.y = last_goal.goal.target_pose.pose.position.y
    # return the response object
    print("last goal service called, last goal position: x = " + str(response.x) + " y = " + str(response.y))
    
    return response

def sub_callback(PlanningActionGoal):
    """
    Callback function for the subscriber /reaching_goal/goal.

    Args:
        PlanningActionGoal (assignment_2_2023.msg.PlanningActionGoal): The goal position of the robot.
    
    Returns:
        None

    This callback function is used by the subscriber to the topic /reaching_goal/goal to save the last goal position in a global variable, so that the service callback function
    can acces the variable at any time and return the last goal position as a response to the service call.
    """
    # save the variable of the last goal in the global variable
    last_goal.goal.target_pose.pose.position.x = PlanningActionGoal.goal.target_pose.pose.position.x
    last_goal.goal.target_pose.pose.position.y = PlanningActionGoal.goal.target_pose.pose.position.y


def lastGoal_srv():
    """
    lastGoal_srv main function.

    Args:
        None

    Returns:    
        None

    This function initializes the node, the subscriber and the service. The node listens on the topic /reaching_goal/goal and when a new goal is set by the user the subcriber callback function is called and the 
    value of the coordinates is saved in the global variable last_goal. The lastGoal service is used to get the coordinates of the last goal set by the user.
    """
    # node initialization
    rospy.init_node('lastGoal_srv')
    
    sub = rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, sub_callback)
   
    Service = rospy.Service('/lastGoal', Last_goal, srv_callback)
    print("last goal service ready")
    
    rospy.spin()
    
    
if __name__ == "__main__":
    try:
        lastGoal_srv()
    except rospy.ROSInterruptException:
        pass
