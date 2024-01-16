#! /usr/bin/env python

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