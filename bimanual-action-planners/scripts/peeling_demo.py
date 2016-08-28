#! /usr/bin/env python

# Script for testing PLN2CTRL client 
import roslib; roslib.load_manifest('bimanual_action_planners')
import rospy
import numpy
import actionlib

# Import the messages
import bimanual_action_planners.msg
import tf
import geometry_msgs.msg
from kuka_fri_bridge.msg    import JointStateImpedance
 

def query_peeling_attractors():
	# Compute Attractors for Reach to Peel/Peel Action				

	# Phase 1 ==== Reach to Peel ===
	lA_p1_attr = geometry_msgs.msg.Transform()
	lA_p1_attr.translation.x = -0.115
	lA_p1_attr.translation.y = -0.10 
	lA_p1_attr.translation.z = 0.287
	lA_p1_attr.rotation.x    = 0.112 
	lA_p1_attr.rotation.y    = 0.949 
	lA_p1_attr.rotation.z    = 0.293 
	lA_p1_attr.rotation.w    = -0.037

	# Phase 2 ==== Peel ===
	lA_p2_attr = geometry_msgs.msg.Transform()
	lA_p2_attr.translation.x = -0.096 
	lA_p2_attr.translation.y = -0.083  	
	lA_p2_attr.translation.z = 0.015
	lA_p2_attr.rotation.x    = 0.112 
	lA_p2_attr.rotation.y    = 0.949 
	lA_p2_attr.rotation.z    = 0.293 
	lA_p2_attr.rotation.w    = -0.037

	return lA_p1_attr,lA_p2_attr

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

def execute_peeling_planner():
	
	#Task Frame in world
	task_frame = geometry_msgs.msg.Transform()
	task_frame.translation.x = -0.403
	task_frame.translation.y = -0.426
	task_frame.translation.z = 0.013
	task_frame.rotation.x = 0
	task_frame.rotation.y = 0
	task_frame.rotation.z = 0
	task_frame.rotation.w = 1

	print "\n\n= = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = ="
	raw_input('Press Enter to Run Bimanual REACH with Coordinated Reaching DS')
	print "\n\n= = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = ="

	# Phase 0 ==== Initial Reach ===

	# Phase 0 Right Arm Attractor in Task RF
	rA_p0_attr = geometry_msgs.msg.Transform()
	rA_p0_attr.translation.x = -0.118
	rA_p0_attr.translation.y = 0.080 
	rA_p0_attr.translation.z = 0.248
	rA_p0_attr.rotation.x    = 0.775 
	rA_p0_attr.rotation.y    = 0.521 
	rA_p0_attr.rotation.z    = -0.157             
	rA_p0_attr.rotation.w    = 0.320

	# Phase 0 Left Arm Attractor in Task RF
	lA_p0_attr = geometry_msgs.msg.Transform()	      
	lA_p0_attr.translation.x = -0.128 
	lA_p0_attr.translation.y = -0.306  
	lA_p0_attr.translation.z = 0.104
	lA_p0_attr.rotation.x    = 0.049  
	lA_p0_attr.rotation.y    = 0.827 
	lA_p0_attr.rotation.z    = 0.557 
	lA_p0_attr.rotation.w    = -0.052

	# Reach with Coordinated DS
	action_type = 'BIMANUAL_REACH'  
	result = send_goal(action_type, 'phase0', task_frame, rA_p0_attr, lA_p0_attr, 10)
	print "Result:"		
	print result.success

	num_peels = 0
	max_peels = 1

	while num_peels < max_peels:

		print "\n\n= = = = = = = = = = = = = = = = = = = = = = = = = ="
		raw_input('Press Enter to Run Reach To Peel with Coupled CDS')
		print "\n\n= = = = = = = = = = = = = = = = = = = = = = = = = ="

		lA_p1_attr,lA_p2_attr = query_peeling_attractors()

		# Reach To Peel with Coupled CDS
		action_type = 'COUPLED_LEARNED_MODEL'  
		result = send_goal(action_type, 'phase1', task_frame, rA_p0_attr, lA_p1_attr, 10)
		print "Result:"		
		print result.success

		print "\n\n= = = = = = = = = = = = = = = = = = ="
		raw_input('Press Enter To Peel with Coupled CDS')
		print "\n\n= = = = = = = = = = = = = = = = = = ="

		# Reach To Peel with Coupled CDS
		action_type = 'COUPLED_LEARNED_MODEL'  
		result = send_goal(action_type, 'phase2', task_frame, rA_p0_attr, lA_p2_attr, 10)
		print "Result:"		
		print result.success

		num_peels = num_peels + 1

	print "\n\n= = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = ="
	raw_input('Press Enter to Run Bimanual RETRACT with Coordinated Reaching DS')
	print "\n\n= = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = ="

	# Phase 4 ==== Retract ===

	# Phase 4 Right Arm Attractor in Task RF
	rA_p4_attr = geometry_msgs.msg.Transform()
	rA_p4_attr.translation.x = -0.104
	rA_p4_attr.translation.y = 0.498 	
	rA_p4_attr.translation.z = 0.574
	rA_p4_attr.rotation.x    = 0.884 
	rA_p4_attr.rotation.y    = 0.282 
	rA_p4_attr.rotation.z    = -0.133
	rA_p4_attr.rotation.w    = 0.348 


	# Phase 4 Left Arm Attractor in Task RF
	lA_p4_attr = geometry_msgs.msg.Transform()
	lA_p4_attr.translation.x = -0.078 
	lA_p4_attr.translation.y = -0.434 	
	lA_p4_attr.translation.z = 0.504
	lA_p4_attr.rotation.x    = 0.112 
	lA_p4_attr.rotation.y    = 0.949 
	lA_p4_attr.rotation.z    = 0.293 
	lA_p4_attr.rotation.w    = -0.037
	
	# Reach with Decoupled DS
	action_type = 'BIMANUAL_REACH'  
	result = send_goal(action_type, 'phase4',  task_frame, rA_p4_attr, lA_p4_attr, 10)
	print "Result:"
	print result.success



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

		#Execute Action Planner for Peeling Task
        execute_peeling_planner()

    except rospy.ROSInterruptException:
        print "program interrupted before completion"