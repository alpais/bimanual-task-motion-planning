#! /usr/bin/env python

# Script for testing PLN2CTRL client 
import roslib; roslib.load_manifest('lasa_action_planners')
import rospy
import numpy
# Import the SimpleActionClient
import actionlib

# Import the messages
import bimanual_action_planners.msg
from robohow_common_msgs.msg import MotionPhase
from robohow_common_msgs.msg import MotionModel
import tf
import geometry_msgs.msg

def PLAN2CTRL_client(action_type, phase, task_frame, right_attractor_frame, left_attractor_frame, timeout):
    # Creates the SimpleActionClient, passing the type of action to the constructor.
    client = actionlib.SimpleActionClient('bimanual_plan2ctrl', bimanual_action_planners.msg.PLAN2CTRLAction)
    print "Phase:", phase
    print "Task Frame: ", task_frame
    print "Right Attractor Frame:", right_attractor_frame
    print "Left Attractor Frame:", left_attractor_frame
    print "Timeout: ", timeout
   
    #Waits until the action server has started up and started listening for goals.
    print "waiting for server"
    client.wait_for_server()
    
    #-----------------------------------------------#
    #----- Set of Goals for the Motion Planner -----#
    #-----------------------------------------------#    
    goal = bimanual_action_planners.msg.PLAN2CTRLGoal(action_type= action_type, action_name = phase, task_frame = task_frame, right_attractor_frame = right_attractor_frame, left_attractor_frame = left_attractor_frame, timeout = timeout)
    
    
    # Sends the goal to the action server.
    print "sending goal", goal
    client.send_goal(goal)
    
    # Waits for the server to finish performing the action.
    print "waiting for result"
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('plan2ctrl_client')


	#Object Frame in world
	task_frame = geometry_msgs.msg.Transform()
	task_frame.translation.x = -0.500
	task_frame.translation.y = -0.600
	task_frame.translation.z = 0.000
	task_frame.rotation.x = 0
	task_frame.rotation.y = 0
	task_frame.rotation.z = 0
	task_frame.rotation.w = 1


	# Phase 1 Right Arm Attractor in Task RF
	rA_p1_attr = geometry_msgs.msg.Transform()
	rA_p1_attr.translation.x = 0.022
	rA_p1_attr.translation.y = 0.136
	rA_p1_attr.translation.z = 0.178
	rA_p1_attr.rotation.x    = 0.033
	rA_p1_attr.rotation.y    = 0.112
	rA_p1_attr.rotation.z    = -0.473
	rA_p1_attr.rotation.w    = 0.719

	# Phase 1 Left Arm Attractor in Task RF
	lA_p1_attr = geometry_msgs.msg.Transform()	      
	lA_p1_attr.translation.x = -0.03
	lA_p1_attr.translation.y = -0.0604
	lA_p1_attr.translation.z = 0.1695
	lA_p1_attr.rotation.x    = 0.96
	lA_p1_attr.rotation.y    = -0.242
	lA_p1_attr.rotation.z    = -0.051
	lA_p1_attr.rotation.w    = -0.53


	# Phase 2 Right Arm Attractor in Task RF
	rA_p2_attr = geometry_msgs.msg.Transform()
	rA_p2_attr.translation.x = 0.034
	rA_p2_attr.translation.y = 0.462	
	rA_p2_attr.translation.z = 0.361
	rA_p2_attr.rotation.x    = -0.189
	rA_p2_attr.rotation.y    = -0.264
	rA_p2_attr.rotation.z    = -0.449
	rA_p2_attr.rotation.w    = 0.833

	# Phase 2 Left Arm Attractor in Task RF
	lA_p2_attr = geometry_msgs.msg.Transform()
	lA_p2_attr.translation.x = 0.054
	lA_p2_attr.translation.y = -0.408	
	lA_p2_attr.translation.z = 0.346
	lA_p2_attr.rotation.x    = 0.960
	lA_p2_attr.rotation.y    = -0.242
	lA_p2_attr.rotation.z    = -0.051
	lA_p2_attr.rotation.w    = -0.130	



	print "\n\n= = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = ="
	raw_input('Press Enter to Run Bimanual Task with Coordinated Reaching DS')
	print "\n\n= = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = ="
	# action_type = 'DECOUPLED_LEARNED_MODEL'  

	# result = PLAN2CTRL_client(action_type, 'phase1', task_frame, rA_p1_attr, lA_p1_attr, 10)
	# print "Result:"		
	# print result.success
	
	# #Wait a few seconds before going back
	# rospy.sleep(1.)
	# result = PLAN2CTRL_client(action_type, 'phase2',  task_frame, rA_p2_attr, lA_p2_attr, 10)
	# print "Result:"
	# print result.success


	action_type = 'BIMANUAL_DS'  
	result = PLAN2CTRL_client(action_type, '', task_frame, rA_p1_attr, lA_p1_attr, 10)
	print "Result:"		
	print result.success
	

    except rospy.ROSInterruptException:
        print "program interrupted before completion"