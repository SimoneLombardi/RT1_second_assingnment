#! /usr/bin/env python

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