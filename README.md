# RT1_second_assingnment


## How to run 
Given you already have a noetic ROS full installation, with Gazebo and Rviz fully funcitonal, the process to run the code is simple:
start with installing "xterm" with the following command:
```
apt-get install xterm
```
add "sudo" in front if needed

The next step reach your ROS workspace and in the src folder use the command:
```
git clone https://github.com/SimoneLombardi/assignment_2_2023.git
```
Build the work space by using 
```
catkin_make
```
in the root folder of your workspace.
Lastly to run the code use
```
roslaunch RT1_second_assingnment assignment1.launch
```
!!! Known issiue !!!
if the python scripts first line returns an error like: #! /usr/bin/env python/r, the reason is the different convention used by the different os (windows and linux), so solve the problem is sufficient to use the command "dos2unix scripts_name.py" that converts all the invisible char in the correct form.

## Node description
### Node A
The node A which is named "bug_ac.py" implements an Action client for the action /reachin_goal, the server for this action was provided to us and is located in the script "bug_as.py", logic for the action client is explained in the following flow chart: 

![Node_A drawio](https://github.com/SimoneLombardi/RT1_second_assingnment/assets/146358714/d3e32265-bcc7-4adc-b7ea-2376cc2791ef)

The node execute two different task: the first one is to control the action client, asking the user to insert two numbers that are used to send a goal request to the action server. Secondly the action client implements a subscriber to the topic /odom, from this topic it recovers some information that are than published on the topic /custom_vel as a custom message definede in Cstm_vel.msg.

The topic /custom_vel is than used by a custom service to compute the average speed of the robot and the distance from the targhet
### Node B
The node B is the service client for the custom service defined in Last_goal.srv, this node implements a subscriber to the topic /reaching_goal/goal, when a new goal is sent to the server the node B saves the information of the position of this new goal.
the service can be called by using:
```
rosservice call /lastgoal
```
### Node C
The last node is used to compute the average velocity of the last 10 known speed of the robot, 10 is the averaging window defined as a parameter in the assignment1.launch file. This node is subscribed to the topic /custom_vel and uses the information provided to compute the average speed. Each time some new data is published on /custom_vel the node C takes the new information, after ten data has been sent the node starts discarding the oldest informaion.

When the service is called by using:
```
rosservice call /divAvSpeed_srv
```
the node automatically call the service provided by node B than using the information computes the euclidean distance from the last targher and the average speed on the last 10 speeds recived from the topic /custom_vel, than prints the results to the user.

### Possible improvements
The use of a subscriber for the Node B is not very efficient, it would be possible to send the information for the service using the function rospy.get_param("des_pos_x") and rospy.get_param("des_pos_y").
The logic of the pathfinding algorithm can be improved also, since the robot follow the obstacle always on the right it ends up not being always optimal. Of course saveing the information of the environmento to create a map is an idea but also using the laser to find the edge of the obstacle to be able to choose from which side is best to approach the obstacle is for sure an improvement.

