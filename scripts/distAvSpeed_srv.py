#! /usr/bin/env python3
"""
.. module:: distAvSpeed_srv

.. moduleauthor:: Simone Lombardi

This node implements the server for the service distAvSpeed_srv. This module calculates the average speed of the robot and the distance between the robot and the last goal.
By subscribing to the topic /custom_vel, it saves the speed of the robot in two lists, one for the linear speed and one for the angular speed. When the lists are full, the module calculates the average speed and the distance between the robot and the last goal and returns them as a response to the service call.
The averaging window is set by the parameter avg_win, that is present in the launch file for the simulation, after the window is full the new data will replace the oldest one, coming from the subscriber /custom_vel.
The module also calls the service lastGoal to get the position of the last goal.

Subscriber:
    /custom_vel: The velocity of the robot, uses the custom message Cstm_vel.

Publisher:
    None

Service:
    distAvSpeed_srv: The service that returns the average speed of the robot and the distance between the robot and the last goal.

Service client:
    lastGoal: The service that returns the position of the last goal.
"""

import rospy
import numpy 

import assignment_2_2023.msg

from assignment_2_2023.srv import Dst_avSpeed, Dst_avSpeedResponse
from assignment_2_2023.srv import Last_goal, Last_goalResponse
from assignment_2_2023.msg import Cstm_vel

counter = 0

Speed_lin = [] # lista vuota per vel lineare
Speed_ang = [] # lista vuota per vel angolare
dr_x = 0
dr_y = 0


def sub_callback(Cstm_vel):
    """
    Callback function for the subscriber /custom_vel.

    Args:
        Cstm_vel (assignment_2_2023.msg.Cstm_vel): The velocity of the robot.

    Returns:
        None

    This callback function is used by the subscriber to the topic /custom_vel to periodically update the data in the lists containing the linear and angular speed of the robot.
    While the service is waiting for a call by the user. The linear speed is saved in Speed_lin and the angular speed is saved in Speed_ang.
    """
    # ogni volta che arriva un messaggio sul topic /custom_vel devo salvare la distanza e dopo che viene salvato
    # 10 dati sulla velocità faccio la media
    global counter
    counter = counter + 1 # incremento il contatore
    
    if counter >= rospy.get_param('avg_win'):
        del Speed_lin[0]
        del Speed_ang[0]
        
        Speed_lin.append(Cstm_vel.vel_x)
        Speed_ang.append(Cstm_vel.vel_z)
        
    elif counter < rospy.get_param('avg_win'):
        
        Speed_lin.append(Cstm_vel.vel_x)
        Speed_ang.append(Cstm_vel.vel_z)
        
    # salvo la poszione del drone 
    dr_x = Cstm_vel.x
    dr_y = Cstm_vel.y
    
        
def srv_callback(Dst_avSpeed):
    """
    Callback function for the service distAvSpeed_srv.

    Args:
        Dst_avSpeed (assignment_2_2023.srv.Dst_avSpeed): The service request.

    Returns:
        Dst_avSpeedResponse: The service response.

    This callback function is used by the service server after recieving a call from the user. It calculates the average speed of the robot and the distance between the robot and the last goal and returns them as a response to the service call.
    """
    # quando viene chiamato il mio messaggio richiamo il servizio per prendere la posizione dell'ultimo goal
    response = srv_client.call()
    
    # definisco la risposta del servizio distAvSpeed_srv
    srvResponse = Dst_avSpeedResponse()
    
    # distanza euclidea tra la posizione del drone e l'ultimo goal
    srvResponse.dist = numpy.sqrt((response.x - dr_x)**2 + (response.y - dr_y)**2)
    
    # velocità media lineare e angolare
    srvResponse.avg_speed_lin = numpy.mean(Speed_lin)
    srvResponse.avg_speed_ang = numpy.mean(Speed_ang)
    
    print(srvResponse)
    
    return srvResponse
    
    
def divAvSpeed_srv():
    """
    divAvSpeed_srv is the main function of the module distAvSpeed_srv.

    Args:
        None
    
    Returns:
        None

    This function initializes the node, the subscriber and the service. It also waits for the service lastGoal to be activated and defines the service proxy.
    The lastGoal service is used to get the coordinates of the last goal set by the user.
    """
    # node inizialization
    rospy.init_node('divAvSpeed_srv')
    
    global sub
    sub = rospy.Subscriber('/custom_vel', Cstm_vel, sub_callback)
    
    # Service definition
    srv = rospy.Service('divAvSpeed_srv', Dst_avSpeed, srv_callback)
    print("dist avrg Speed service ready")
    
    # wait for service activation
    rospy.wait_for_service('lastGoal')
    
    # define service proxy
    global srv_client
    srv_client = rospy.ServiceProxy('lastGoal', Last_goal)
    
    
    # repetition
    rospy.spin()
    
    
    

if __name__ == "__main__":
    try:
        divAvSpeed_srv()
    except rospy.ROSInterruptException:
        pass
