#! /usr/bin/env python
"""
.. module:: bug_ac

.. moduleauthor:: Simone Lombardi

This node implements the client for the action server bug_as.py. This module asks the user for a couple of goal cordinates and send the request to the action server.
The user can also cancel the goal and insert a new one.

Subscriber:
    /odom: The position and velocity of the robot.

Publisher:
    /custom_vel: The velocity of the robot, uses the custom message Cstm_vel.
"""


import rospy

import math
import time

# needed for the SimpleActionClient
import actionlib
import actionlib.msg
import assignment_2_2023.msg

from geometry_msgs.msg import Point, Pose, Twist
from assignment_2_2023.msg import Cstm_vel
from nav_msgs.msg import Odometry



def callback(Odometry):
    """
    Callback function for the subscriber /odom. 

    Args:
        Odometry (nav_msgs.msg.Odometry): The position and velocity of the robot.

    Returns:
        None

    This callback function reads the position and velocity of the robot from the topic /odom, than saves some of the information in a custom message (Cstm_vel)
    and publishes it on the topic /custom_vel.
    """
    custom_vel = Cstm_vel()
    # define the variable of type Csmt_vel
    
    # get position of the robot
    custom_vel.x = Odometry.pose.pose.position.x
    custom_vel.y = Odometry.pose.pose.position.y
    
    #get velocity of the robot
    custom_vel.vel_x = Odometry.twist.twist.linear.x
    custom_vel.vel_z = Odometry.twist.twist.angular.z
    
    pub.publish(custom_vel)

    
def user_input(client):
    """
    This function asks the user for an input to cancel the goal or to continue to the goal, furthermore it checks if the goal is already reached if so 
    the request to cancel the goal is not sent to the action server.

    Args:
        client (SimpleActionClient): The action client.

    Returns:
        int: 0 if the goal is reached, 1 if the goal is not reached.
    """
    # if the goal is still active i ask the user to cancel it
    inp = input("Do you want to cancel the goal? (y/n) ")
        
    if inp == 'y':
        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            print("Goal aleady reached")
            return 0
        else:
            client.cancel_goal()
            print("Goal canceled")
            return 0
    elif inp == 'n':
        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            print("Goal aleady reached")
            return 0
        else:
            print("Continue to the goal")
            return 1
        
        


def bug_ac():
    """
    bug_ac function the main function of the module.

    Args:
        None

    Returns:
        None

    This function initializes the node, the subscriber(to the topic /odom) and the publisher(to the topic /custom_vel).
    It also initializes the action client and waits for the action server to start, and in a while loop asks the user for a 'goal' and sends the request to the action server.
    """
    rospy.init_node('bug_ac')
    
    sub = rospy.Subscriber('/odom', Odometry, callback)
    # recover information from the topic /odom -> to publish them on another topi'''
    
    global pub
    pub = rospy.Publisher('/custom_vel', Cstm_vel, queue_size=10)
    
    
    # define the action client variable 
    # fields: name of the action server, pkg_name.msg.dotActionfileName/Action
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
    
    # wait for the action server to start
    client.wait_for_server()
    
    # set the goal
    # create a goal variable of msg file PlanningGoal --> il tipo PlanningGoal ha un campo target_pose di tipo Pose
    # che è di tipo geometry_msgs/PoseStamped
    goal = assignment_2_2023.msg.PlanningGoal()
    
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
         
        try:
            # get the user input for the goal
            goal.target_pose.pose.position.x = float(input("Please enter the x coordinate of the goal: "))
            
            goal.target_pose.pose.position.y = float(input("Please enter the y coordinate of the goal: "))
        except ValueError:
            print("Invalid input, please try again")
            continue
            
        
        # send the goal to the action server
        client.send_goal(goal)
        
        # wait to recive the result of the action from the server
        while client.get_state() in [actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PENDING]:
            ret = user_input(client)
            
            if(ret == 0):
                break


        rate.sleep()
        
    rospy.spin()
    
    
if __name__ == '__main__':
    try:
        bug_ac()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)