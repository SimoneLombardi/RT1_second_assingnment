#! /usr/bin/env python

import rospy
import time
import random

import assignment_2_2023.msg

from assignment_2_2023.srv import Last_goal, Last_goalResponse
from assignment_2_2023.msg import PlanningActionGoal

last_goal = assignment_2_2023.msg.PlanningActionGoal()

def srv_callback(request):
    # create a response object
    response = Last_goalResponse()
    # fill the response object with the last goal position
    response.x = last_goal.goal.target_pose.pose.position.x
    response.y = last_goal.goal.target_pose.pose.position.y
    # return the response object
    print("last goal service called, last goal position: x = " + str(response.x) + " y = " + str(response.y))
    
    return response

def sub_callback(PlanningActionGoal):
    # save the variable of the last goal in the global variable
    last_goal.goal.target_pose.pose.position.x = PlanningActionGoal.goal.target_pose.pose.position.x
    last_goal.goal.target_pose.pose.position.y = PlanningActionGoal.goal.target_pose.pose.position.y


def lastGoal_srv():
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