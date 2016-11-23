#include "bimanual_action_server.h"

bool BimanualActionServer::collab_active_model_execution(TaskPhase phase, tf::Transform task_frame, tf::Transform left_att, double dt, CDSController::DynamicsType l_masterType, CDSController::DynamicsType l_slaveType, double reachingThreshold, double orientationThreshold){

    ROS_INFO_STREAM(" Starting Collaborative Execution, human master");
    ROS_INFO_STREAM(" Model Path "                  << model_base_path);
    ROS_INFO_STREAM(" Execute Learned model: phase "<< phase);
    ROS_INFO_STREAM(" Reaching threshold "          << reachingThreshold);
    ROS_INFO_STREAM(" Orientation threshold "       << orientationThreshold);
    ROS_INFO_STREAM(" Model DT "                    << model_dt);

    tf::Transform  left_final_target;
    tf::Transform human_final_target; human_final_target.setIdentity();

    // ============= Left Arm relative to the master arm ===========

    // Get Right arm position from vision (i.e. the wrist)
    // don't care at all about the bowl here, we assume the right arm positions wrt the bowl


    tf::Transform wrist_in_robot_base; wrist_in_robot_base.setIdentity(); // >> UPDATE FROM VISION

    tf::Transform fixed_right_arm_rf;
    fixed_right_arm_rf.setIdentity();
    fixed_right_arm_rf.setRotation(tf::Quaternion(0.807, 0.467, -0.145, 0.332));
    fixed_right_arm_rf.setOrigin(r_ee_pose.getOrigin());

    if(task_id == SCOOPING_TASK_ID && (phase == PHASE_SCOOP_REACH_TO_SCOOP || phase == PHASE_SCOOP_SCOOP) ){

        tf::Transform fixed_right_arm_rf;
        fixed_right_arm_rf.setIdentity();
        fixed_right_arm_rf.setRotation(r_ee_pose.getRotation());
        fixed_right_arm_rf.setOrigin(r_ee_pose.getOrigin());

        // To determine ATT run  >> rosrun tf tf_echo /TOOL_ft /Hand_ft
        if (phase == PHASE_SCOOP_REACH_TO_SCOOP){

            tf::Transform fixed_reach_to_scoop_att;

            fixed_reach_to_scoop_att.setOrigin(tf::Vector3(-0.001-0.02, -0.07, 0.306)); // from tf echo
            fixed_reach_to_scoop_att.setRotation(tf::Quaternion(0.813, -0.458, 0.159, -0.324));

            left_final_target.mult(fixed_right_arm_rf, fixed_reach_to_scoop_att);

        } else if (phase == PHASE_SCOOP_SCOOP){

            tf::Transform fixed_scoop_att;

            fixed_scoop_att.setOrigin(tf::Vector3(-0.047-0.01, -0.064, 0.270));
            fixed_scoop_att.setRotation(tf::Quaternion(0.312, 0.903, 0.262, 0.138));

            left_final_target.mult(fixed_right_arm_rf, fixed_scoop_att);

        } else if (phase == PHASE_SCOOP_DEPART){

            tf::Transform fixed_depart_att;

            fixed_depart_att.setOrigin(tf::Vector3(0.123, -0.242, 0.273));
            fixed_depart_att.setRotation(tf::Quaternion(-0.123, 0.970, 0.190, 0.085));

            left_final_target.mult(fixed_right_arm_rf, fixed_depart_att);

        } else if (phase == PHASE_SCOOP_TRASH){

            tf::Transform fixed_trash_att;
            fixed_trash_att.setOrigin(tf::Vector3(-0.073, -0.221, 0.154));
            fixed_trash_att.setRotation(tf::Quaternion(0.949, -0.036, 0.097, -0.297));

            left_final_target.mult(fixed_right_arm_rf, fixed_trash_att);

        } else if (phase == PHASE_SCOOP_RETRACT){

            tf::Transform fixed_away_att;

            fixed_away_att.setOrigin(tf::Vector3(-0.031, -0.807, 0.992));
            fixed_away_att.setRotation(tf::Quaternion(-0.009, 0.963, 0.226, -0.147));

            left_final_target.mult(fixed_right_arm_rf, fixed_away_att);
        }

    }
    else // >> Set the target of the left arm relative to the task frame
        left_final_target.mult(task_frame, left_att); // final target in world

    // ======================================================================================================
    // ========= Initialize CDS models
    // ======================================================================================================

    // Model Task Frame
    tf::Transform model_task_frame;
    model_task_frame.setIdentity();


    // Setting Initial conditions
    // (For Visualization of Trajectories)
    if (initial_config == true){
        r_curr_ee_pose = vision_wrist_pose; // >>> COMPUTE IT IN ROB FRAME
        l_curr_ee_pose = l_ee_pose;
    }

    // Initialize CDS for left arm moving wrt the right arm
    CDSExecution *left_cdsRun = new CDSExecution;
    left_cdsRun->initSimple(model_base_path, phase, L_ARM_ID, L_ARM_ROLE);
    left_cdsRun->setObjectFrame(toMatrix4(model_task_frame));
    left_cdsRun->setAttractorFrame(toMatrix4(left_final_target));
    left_cdsRun->setCurrentEEPose(toMatrix4(l_curr_ee_pose));
    left_cdsRun->setDT(model_dt);
    left_cdsRun->setMotionParameters(l_ori_gain, l_pos_gain, l_err_gain, reachingThreshold, l_masterType, l_slaveType);
    left_cdsRun->postInit();


    // Initialize coupling model

    // Variables for execution
    ros::Duration loop_rate(model_dt);
    tf::Pose l_mNextRobotEEPose = l_curr_ee_pose;


    // ======================================================================================================
    // ========= Real time loop
    // ======================================================================================================

    ROS_INFO("Execution started");
    while(ros::ok()) {

        if (initial_config == true){
            // Setting Initial conditions
            r_curr_ee_pose = wrist_in_robot_base;
            l_curr_ee_pose = l_ee_pose;

            // Compensating for the trajectory modifications due to the desired force
            if (bUseForce_l_arm)
                l_curr_ee_pose = remove_correction_due_to_force_from_trajectory(L_ARM_ID, force_control_axis);
        }
        else{
            // For visualization of trajectories
            r_curr_ee_pose = wrist_in_robot_base;
            l_curr_ee_pose = l_des_ee_pose;
        }

        // Publish attractors if running in simulation or with fixed values
        publish_task_frames(r_curr_ee_pose, l_curr_ee_pose, human_final_target, left_final_target, task_frame);

        // Current progress variable (position/orientation error).
        //        r_pos_err = (right_final_target.getOrigin() - r_curr_ee_pose.getOrigin()).length();
        l_pos_err = (left_final_target.getOrigin()  - l_curr_ee_pose.getOrigin()).length();

        //Real Orientation Error qdiff = acos(dot(q1_norm,q2_norm))*180/pi
        //        r_ori_err = acos(abs(right_final_target.getRotation().dot(r_curr_ee_pose.getRotation())));
        l_ori_err = acos(abs(left_final_target.getRotation().dot(l_ee_pose.getRotation())));

        if (bDisplayDebugInfo){
            //          ROS_INFO_STREAM_THROTTLE(0.5,"Position Threshold : "    << reachingThreshold    << " ... Current Right Error: " << r_pos_err << " Left Error: " << l_pos_err);
            ROS_INFO_STREAM_THROTTLE(0.5,"Orientation Threshold : " << orientationThreshold << " ... Current Right Error: " << r_ori_err << " Left Error: " << l_ori_err);
        }


        // Update coupling and CDS parameters


        //  >>> Cartesian Trajectory Computation <<<

        // Update Attractor from Vision

        // Compute Next Desired EE Pose for Left Arm
        left_cdsRun->setCurrentEEPose(toMatrix4(l_mNextRobotEEPose));
        toPose(left_cdsRun->getNextEEPose(), l_mNextRobotEEPose);

        l_des_ee_pose = l_mNextRobotEEPose;
        l_des_ee_pose.setRotation(l_curr_ee_pose.getRotation().slerp(left_final_target.getRotation(), 0.25) );


        // >>> Force computation <<<
        if (bUseForce_l_arm)
            l_des_ee_pose = update_ee_pose_based_on_force(L_ARM_ID, force_control_axis);

        // Make next pose the current pose for open-loop simulation
        if (just_visualize==true)
            initial_config=false;

        //******************************//
        //  Send the computed ee poses  //
        //******************************//

        if (just_visualize==false){
            filter_arm_motion(r_des_ee_pose, l_des_ee_pose);         // Filter Commands in Cartesian space
            sendPoseLeft(l_des_ee_pose);
        }

        as_.publishFeedback(feedback_);

        // >>>> Check Convergence in Position
        if(l_pos_err < reachingThreshold){

            ROS_INFO("POSITION DYNAMICS CONVERGED!");

            if (bIgnoreOri){
                sendPoseLeft(l_curr_ee_pose); // Stop the robot after the position has converged
                break;
            }

            // >>>> Check if contact should be established at the end of the action
            if (bEndInContact_l_arm){
                if (bWaitForForces_left_arm){
                    ROS_INFO_STREAM("In PHASE " << phase << " >> searching for contact now on arm " << L_ARM_ID);
                    bool x_l_arm = find_object_by_contact(L_ARM_ID, search_axis_l_arm, max_search_distance_l_arm, max_vertical_speed_l_arm, max_contact_force_l_arm);
                    return x_l_arm;
                }
                ROS_INFO("Finished Finding Object LOOP");
                break;
            }


            // >>>> Check Convergence in Orientation
            if((l_ori_err < orientationThreshold) || isnan(l_ori_err)){
                ROS_INFO("LEFT ORIENTATION DYN CONVERGED!");
                sendPoseLeft(l_curr_ee_pose); // Stop the robot by sending the current position again, which results in zero velocity
                break;
            }
        }

        loop_rate.sleep();
    }
    ROS_INFO("OUT OF LOOP CDS");

    delete left_cdsRun;

    delete l_cdd_cart_filter;


}


