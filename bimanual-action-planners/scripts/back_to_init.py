#! /usr/bin/env python

# Script for testing PLN2CTRL client 
import roslib; roslib.load_manifest('bimanual_action_planners')
import rospy
import numpy
# Import the SimpleActionClient
import actionlib

# Import the messages
import bimanual_action_planners.msg
# from robohow_common_msgs.msg import MotionPhase
# from robohow_common_msgs.msg import MotionModel
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


	#Task Frame in world
	task_frame = geometry_msgs.msg.Transform()
	task_frame.translation.x = -0.403
	task_frame.translation.y = -0.426
	task_frame.translation.z = 0.013
	task_frame.rotation.x = 0
	task_frame.rotation.y = 0
	task_frame.rotation.z = 0
	task_frame.rotation.w = 1


	# Phase 2 ==== Retract ===

	# Phase 2 Right Arm Attractor in Task RF
	rA_p2_attr = geometry_msgs.msg.Transform()
	rA_p2_attr.translation.x = -0.104
	rA_p2_attr.translation.y = 0.498 	
	rA_p2_attr.translation.z = 0.574
	rA_p2_attr.rotation.x    = 0.884 
	rA_p2_attr.rotation.y    = 0.282 
	rA_p2_attr.rotation.z    = -0.133
	rA_p2_attr.rotation.w    = 0.348 


	# Phase 2 Left Arm Attractor in Task RF
	lA_p2_attr = geometry_msgs.msg.Transform()
	lA_p2_attr.translation.x = -0.078 
	lA_p2_attr.translation.y = -0.434 	
	lA_p2_attr.translation.z = 0.394
	lA_p2_attr.rotation.x    = 0.112 
	lA_p2_attr.rotation.y    = 0.949 
	lA_p2_attr.rotation.z    = 0.293 
	lA_p2_attr.rotation.w    = -0.037

	print "\n\n= = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = ="
	raw_input('Press Enter to Run Bimanual RETRACT with Coordinated Reaching DS')
	print "\n\n= = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = ="


	# Reach with VO DS
	action_type = 'BIMANUAL_REACH'  
	result = PLAN2CTRL_client(action_type, 'phase4',  task_frame, rA_p2_attr, lA_p2_attr, 10)
	print "Result:"
	print result.success

    except rospy.ROSInterruptException:
        print "program interrupted before completion"