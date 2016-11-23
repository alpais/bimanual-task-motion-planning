#! /usr/bin/env python

# Script for testing PLN2CTRL client 
import roslib; roslib.load_manifest('bimanual_action_planners')
import sys
import rospy
import numpy
import actionlib

# Import the messages
import bimanual_action_planners.msg
import tf
import geometry_msgs.msg
from kuka_fri_bridge.msg    import JointStateImpedance

 
def query_reach_attractors():

	# Phase 0 Right Arm Attractor in Task RF >> Right Arm reaches directly in the good pose
	rA_p0_attr = geometry_msgs.msg.Transform()
	rA_p0_attr.translation.x = -0.151 
	rA_p0_attr.translation.y = -0.002 
	rA_p0_attr.translation.z =  0.260
	rA_p0_attr.rotation.x    =  0.357   
	rA_p0_attr.rotation.y    = -0.101 
	rA_p0_attr.rotation.z    =  0.276
	rA_p0_attr.rotation.w    =  0.887


	# Phase 0 Left Arm Attractor in Task RF
	lA_p0_attr = geometry_msgs.msg.Transform()
	lA_p0_attr.translation.x = -0.090 
	lA_p0_attr.translation.y = -0.255  
	lA_p0_attr.translation.z =  0.422
	lA_p0_attr.rotation.x    = -0.643  
	lA_p0_attr.rotation.y    =  0.290
	lA_p0_attr.rotation.z    =  0.339
	lA_p0_attr.rotation.w    =  0.622

	return rA_p0_attr, lA_p0_attr

def query_scoop_attractors():

	# Phase 2 ==== Scoop ===
	rA_p1_attr = geometry_msgs.msg.Transform()
	rA_p1_attr.translation.x = -0.151 
	rA_p1_attr.translation.y = -0.002 
	rA_p1_attr.translation.z =  0.260
	rA_p1_attr.rotation.x    =  0.357   
	rA_p1_attr.rotation.y    = -0.101 
	rA_p1_attr.rotation.z    =  0.276
	rA_p1_attr.rotation.w    =  0.887

	# Phase 2 ==== Scoop ===
	lA_p1_attr = geometry_msgs.msg.Transform()
	lA_p1_attr.translation.x = -0.120 
	lA_p1_attr.translation.y = -0.245237
	lA_p1_attr.translation.z =  0.40396
	lA_p1_attr.rotation.x    = -0.233  
	lA_p1_attr.rotation.y    =  0.779
	lA_p1_attr.rotation.z    =  0.576
	lA_p1_attr.rotation.w    =  0.087

	return rA_p1_attr,lA_p1_attr


def query_depart_attractors():

	# Phase 3 ==== Depart and reach on top of the bowl ===
	rA_p2_attr = geometry_msgs.msg.Transform()
	rA_p2_attr.translation.x = -0.219  
	rA_p2_attr.translation.y =  0.115 
	rA_p2_attr.translation.z =  0.250
	rA_p2_attr.rotation.x    =  0.353  
	rA_p2_attr.rotation.y    = -0.153
	rA_p2_attr.rotation.z    =  0.210
	rA_p2_attr.rotation.w    =  0.899

	lA_p2_attr = geometry_msgs.msg.Transform()
	lA_p2_attr.translation.x = -0.030
	lA_p2_attr.translation.y = -0.203
	lA_p2_attr.translation.z =  0.357
	lA_p2_attr.rotation.x    = -0.314 
	lA_p2_attr.rotation.y    =  0.766
	lA_p2_attr.rotation.z    =  0.512
	lA_p2_attr.rotation.w    = 0.229

	return rA_p2_attr,lA_p2_attr	


def query_trash_attractors():

	# Trash - Right Arm Attractor in Task RF
	rA_p3_attr = geometry_msgs.msg.Transform()
	rA_p3_attr.translation.x = -0.270  
	rA_p3_attr.translation.y =  0.338
	rA_p3_attr.translation.z =  0.30239
	rA_p3_attr.rotation.x    =  0.352 
	rA_p3_attr.rotation.y    = -0.196
	rA_p3_attr.rotation.z    =  0.083
	rA_p3_attr.rotation.w    =  0.911
 
	# Trash - Left Arm Attractor in Task RF
	lA_p3_attr = geometry_msgs.msg.Transform()
	lA_p3_attr.translation.x = -0.057 
	lA_p3_attr.translation.y = -0.206
	lA_p3_attr.translation.z =  0.28211
	lA_p3_attr.rotation.x    = -0.738 
	lA_p3_attr.rotation.y    =  0.307
	lA_p3_attr.rotation.z    =  0.247
	lA_p3_attr.rotation.w    =  0.548
	
	return rA_p3_attr, lA_p3_attr

def query_away_attractors():
        # Phase 4 ==== Away ===

	# Away - Right Arm Attractor in Task RF
	rA_p4_attr = geometry_msgs.msg.Transform()
	rA_p4_attr.translation.x = -0.168
	rA_p4_attr.translation.y =  0.4523
	rA_p4_attr.translation.z =  0.216
	rA_p4_attr.rotation.x    =  0.386 
	rA_p4_attr.rotation.y    = -0.144
	rA_p4_attr.rotation.z    =  0.054
	rA_p4_attr.rotation.w    =  0.910
  
	# Away - Left Arm Attractor in Task RF
	lA_p4_attr = geometry_msgs.msg.Transform()
	lA_p4_attr.translation.x = -0.246 
	lA_p4_attr.translation.y = -0.40753
	lA_p4_attr.translation.z =  0.308
	lA_p4_attr.rotation.x    = -0.152 
	lA_p4_attr.rotation.y    =  0.809
	lA_p4_attr.rotation.z    =  0.568
	lA_p4_attr.rotation.w    = -0.005

	return rA_p4_attr, lA_p4_attr

def send_goal(action_type, phase, task_frame, right_attractor_frame, left_attractor_frame, timeout):
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

def execute_scooping_planner():
	
	#Task Frame in world
	task_frame = geometry_msgs.msg.Transform()
	task_frame.translation.x = -0.403
	task_frame.translation.y = -0.426
	task_frame.translation.z = 0.013
	task_frame.rotation.x = 0
	task_frame.rotation.y = 0
	task_frame.rotation.z = 0
	task_frame.rotation.w = 1

	# print "\n\n= = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = ="
	# raw_input('Press Enter to Run Bimanual REACH with Coordinated Reaching DS')
	# print "\n\n= = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = ="

	# rA_p0_attr, lA_p0_attr = query_init_attractors()

	# Reach with Coordinated DS
	# action_type = 'BIMANUAL_REACH'  
	# result = send_goal(action_type, 'phase0', task_frame, rA_p0_attr, lA_p0_attr, 10)
	# print "Result:"		
	# print result.success
	


	print "\n\n= = = = = = = = = = = = = = = = = = = = = = = = = ="
	raw_input('Press Enter to Run Reach-To-Scoop with Coupled CDS')
	print "\n\n= = = = = = = = = = = = = = = = = = = = = = = = = ="

	rA_p0_attr,lA_p0_attr = query_reach_attractors()
	action_type = 'COLLABORATIVE_ACTIVE'  
	result = send_goal(action_type, 'phase1', task_frame, rA_p0_attr, lA_p0_attr, 10)
	print "Result:"		
	print result.success

	print "\n\n= = = = = = = = = = = = = = = = = = ="
	raw_input('Press Enter To Scoop with Coupled CDS')
	print "\n\n= = = = = = = = = = = = = = = = = = ="

	rA_p1_attr,lA_p1_attr = query_scoop_attractors()
	action_type = 'COLLABORATIVE_ACTIVE'  
	result = send_goal(action_type, 'phase2', task_frame, rA_p1_attr, lA_p1_attr, 10)
	print "Result:"		
	print result.success

	print "\n\n= = = = = = = = = = = = = = = = = = ="
	raw_input('Press Enter To Depart with Coupled CDS')
	print "\n\n= = = = = = = = = = = = = = = = = = ="

	rA_p2_attr,lA_p2_attr = query_depart_attractors()
	action_type = 'COLLABORATIVE_ACTIVE'  
	result = send_goal(action_type, 'phase3', task_frame, rA_p2_attr, lA_p2_attr, 10)
	print "Result:"		
	print result.success

	print "\n\n= = = = = = = = = = = = = = = = = = ="
	raw_input('Press Enter To Trash with Coupled CDS')
	print "\n\n= = = = = = = = = = = = = = = = = = ="

	rA_p3_attr,lA_p3_attr = query_trash_attractors()
	action_type = 'COLLABORATIVE_ACTIVE'  
	result = send_goal(action_type, 'phase4', task_frame, rA_p3_attr, lA_p3_attr, 10)
	print "Result:"		
	print result.success

#	print "\n\n= = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = ="
#	raw_input('Press Enter to Run Bimanual RETRACT with Coordinated Reaching DS')
#	print "\n\n= = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = ="
#	
#	rA_p4_attr, lA_p4_attr = query_away_attractors()
#	action_type = 'COLLABORATIVE_PASSIVE'  
#	result = send_goal(action_type, 'phase5',  task_frame, rA_p4_attr, lA_p4_attr, 10)
#	print "Result:"
#	print result.success


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('plan2ctrl_client')
        
        # Creates the SimpleActionClient, passing the type of action to the constructor.
        client = actionlib.SimpleActionClient('bimanual_plan2ctrl', bimanual_action_planners.msg.PLAN2CTRLAction)

        #Waits until the action server has started up and started listening for goals.
        print "waiting for server"
        client.wait_for_server()

		#Execute Action Planner for Scooping Task
        execute_scooping_planner()

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
