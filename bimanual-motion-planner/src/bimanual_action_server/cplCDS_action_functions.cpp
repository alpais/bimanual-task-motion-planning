#include "bimanual_action_server.h"

bool BimanualActionServer::coupled_learned_model_execution(TaskPhase phase, CDSController::DynamicsType r_masterType, CDSController::DynamicsType r_slaveType,
                                                           CDSController::DynamicsType l_masterType, CDSController::DynamicsType l_slaveType,
                                                           double reachingThreshold, double orientationThreshold, tf::Transform task_frame,
                                                           tf::Transform right_att, tf::Transform left_att)
{

    ROS_INFO_STREAM(" Model Path "                  << model_base_path);
    ROS_INFO_STREAM(" Execute Learned model: phase "<< phase);
    ROS_INFO_STREAM(" Reaching threshold "          << reachingThreshold);
    ROS_INFO_STREAM(" Orientation threshold "       << orientationThreshold);
    ROS_INFO_STREAM(" Model DT "                    << model_dt);


    // ======================================================================================================
    // ========= Computing the target of the right and left arms for each action wrt the given attractors ===
    // ======================================================================================================

    tf::Transform  right_final_target, left_final_target;

    // ============ Right Arm ==========
    // During the Peeling task the Master Arm Stays in the Position/Orientation for all but ROTATE/REACH,
    // here a give a fixed rotation value of 45 degrees on the Z axis
    // (Should be substituted by vision - no time for this now - subscribed to /zuch/feats)
    if (task_id == PEELING_TASK_ID && phase == PHASE_ROTATE){
        right_final_target = r_ee_pose;
        tf::Transform delta_rot; delta_rot.setIdentity();
        right_final_target.mult(r_ee_pose.inverse(),right_final_target);
        delta_rot.setBasis(tf::Matrix3x3(0.707,-0.707,0,0.707,0.707,0,0,0,1)); // Rotate around Z EE RF (45dg)
        delta_rot.setBasis(tf::Matrix3x3(0.866,-0.5,0,0.5,0.866,0,0,0,1)); // Rotate around Z EE RF (30dg)
        right_final_target.mult(delta_rot,right_final_target);
        right_final_target.mult(r_ee_pose,right_final_target);
    } else if (task_id == SCOOPING_TASK_ID && (phase == PHASE_SCOOP_REACH_TO_SCOOP || phase == PHASE_SCOOP_DEPART || phase == PHASE_SCOOP_TRASH)){
        right_final_target.mult(task_frame, right_att);
    }
    else
        right_final_target = r_ee_pose;

    // In the Scooping task the Master Arm maintains its position just in the scooping phase
    // In depart it goes away few cm >> see Att in the python script
    // In Trash it goes away even more

    double dheight = 0.10;
    // ============= Left Arm relative to the master arm ===========
    // Slave Arm has Attractor on Object relative to a Right Fixed RF
    // (Should be substituted by vision (i.e. compute attractors from point cloud) - no time for this now)
    //  >>>>> Peeling TASK <<<<<
    if (task_id == PEELING_TASK_ID && (phase == PHASE_REACH_TO_PEEL || phase == PHASE_PEEL)){
        tf::Transform fixed_right_arm_rf;
        fixed_right_arm_rf.setIdentity();
        fixed_right_arm_rf.setRotation(tf::Quaternion(0.807, 0.467, -0.145, 0.332));
        fixed_right_arm_rf.setOrigin(r_ee_pose.getOrigin());

        if (phase == PHASE_REACH_TO_PEEL){
            tf::Transform fixed_reach_to_peel_attr;

            //- Translation: [-0.130, 0.073, 0.238]
            //- Rotation: in Quaternion [-0.333, 0.753, -0.453, 0.342]

            fixed_reach_to_peel_attr.setOrigin(tf::Vector3(-0.130, 0.073+dheight, 0.238));
            fixed_reach_to_peel_attr.setRotation(tf::Quaternion(-0.333, 0.753, -0.453, 0.342)); //KW
            fixed_reach_to_peel_attr.setRotation(tf::Quaternion(0.6812, -0.0885, 0.7265, 0.0178));
            left_final_target.mult(fixed_right_arm_rf,fixed_reach_to_peel_attr);
        } else if(phase == PHASE_PEEL){

            //- Translation: [-0.155, 0.066, 0.367]
            //- Rotation: in Quaternion [-0.297, 0.738, -0.492, 0.354]

            tf::Transform fixed_peel_attr;
            fixed_peel_attr.setOrigin(tf::Vector3(-0.155, 0.066, 0.367));
            fixed_peel_attr.setRotation(tf::Quaternion(-0.333, 0.753, -0.453, 0.342));
            left_final_target.mult(fixed_right_arm_rf,fixed_peel_attr);

        }
        // >>>>> Scooping TASK <<<<
    } else if(task_id == SCOOPING_TASK_ID && (phase == PHASE_SCOOP_REACH_TO_SCOOP || phase == PHASE_SCOOP_SCOOP) ){

        tf::Transform fixed_right_arm_rf;
        fixed_right_arm_rf.setIdentity();
        fixed_right_arm_rf.setRotation(r_ee_pose.getRotation());
        fixed_right_arm_rf.setOrigin(r_ee_pose.getOrigin());

        // To determine ATT run  >> rosrun tf tf_echo /TOOL_ft /Hand_ft
        if (phase == PHASE_SCOOP_REACH_TO_SCOOP){

            tf::Transform fixed_reach_to_scoop_att;
            double dcorr; dcorr = 0.02;
            fixed_reach_to_scoop_att.setOrigin(tf::Vector3(-0.010-dcorr, -0.087, 0.286)); // from tf echo
            fixed_reach_to_scoop_att.setRotation(tf::Quaternion(-0.325, 0.862, 0.031, 0.387));

            left_final_target.mult(fixed_right_arm_rf, fixed_reach_to_scoop_att);

        } else if (phase == PHASE_SCOOP_SCOOP){

            tf::Transform fixed_scoop_att;

            fixed_scoop_att.setOrigin(tf::Vector3(-0.047, -0.061, 0.270));
            fixed_scoop_att.setRotation(tf::Quaternion(0.312, 0.903, 0.262, 0.138));

            left_final_target.mult(fixed_right_arm_rf, fixed_scoop_att);

        } else if (phase == PHASE_SCOOP_DEPART){

            tf::Transform fixed_depart_att;

            fixed_depart_att.setOrigin(tf::Vector3(0.123, -0.242, 0.273));
            fixed_depart_att.setRotation(tf::Quaternion(-0.123, 0.970, 0.190, 0.085));

            left_final_target.mult(fixed_right_arm_rf, fixed_depart_att);

        } else if (phase == PHASE_SCOOP_TRASH){

            tf::Transform fixed_trash_att;
            double dh = 0.12;
            fixed_trash_att.setOrigin(tf::Vector3(0.175, -0.480 + dh, 0.285));
//            fixed_trash_att.setRotation(tf::Quaternion(0.791, -0.536, -0.216, -0.200));
            fixed_trash_att.setRotation(tf::Quaternion(0.705, -0.625, -0.039, -0.334));
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
        r_curr_ee_pose = r_ee_pose;
        l_curr_ee_pose = l_ee_pose;
    }

    // Initialize CDS for right arm
    CDSExecution *right_cdsRun = new CDSExecution;
    right_cdsRun->initSimple(model_base_path, phase, R_ARM_ID, R_ARM_ROLE);
    right_cdsRun->setObjectFrame(toMatrix4(model_task_frame));
    right_cdsRun->setAttractorFrame(toMatrix4(right_final_target));
    right_cdsRun->setCurrentEEPose(toMatrix4(r_curr_ee_pose));
    right_cdsRun->setDT(model_dt);
    right_cdsRun->setMotionParameters(r_ori_gain, r_pos_gain, r_err_gain, reachingThreshold, r_masterType, r_slaveType);
    right_cdsRun->postInit();


    // Initialize CDS for left arm >>> this is correct only if the left arm moves wrt the right arm
    CDSExecution *left_cdsRun = new CDSExecution;
    left_cdsRun->initSimple(model_base_path, phase, L_ARM_ID, L_ARM_ROLE);
    left_cdsRun->setObjectFrame(toMatrix4(model_task_frame));
    left_cdsRun->setAttractorFrame(toMatrix4(left_final_target));
    left_cdsRun->setCurrentEEPose(toMatrix4(l_curr_ee_pose));
    left_cdsRun->setDT(model_dt);
    left_cdsRun->setMotionParameters(l_ori_gain, l_pos_gain, l_err_gain, reachingThreshold, l_masterType, l_slaveType);
    left_cdsRun->postInit();


    // otherwise use the following initialization wrt the task frame
    //    left_cdsRun->setObjectFrame(toMatrix4(task_frame));
    //    left_cdsRun->setAttractorFrame(toMatrix4(left_att));


    // Variables for execution
    ros::Duration loop_rate(model_dt);
    tf::Pose r_mNextRobotEEPose = r_curr_ee_pose;
    tf::Pose l_mNextRobotEEPose = l_curr_ee_pose;


    // ======================================================================================================
    // ========= Real time loop
    // ======================================================================================================

    ROS_INFO("Execution started");
    while(ros::ok()) {

        if (initial_config == true){
            // Setting Initial conditions
            r_curr_ee_pose = r_ee_pose;
            l_curr_ee_pose = l_ee_pose;

            // Compensating for the trajectory modifications due to the desired force
            if (bUseForce_l_arm)
                l_curr_ee_pose = remove_correction_due_to_force_from_trajectory(L_ARM_ID, force_control_axis);
            if (bUseForce_r_arm)
                r_curr_ee_pose = remove_correction_due_to_force_from_trajectory(R_ARM_ID, force_control_axis);
        }
        else{
            // For visualization of trajectories
            r_curr_ee_pose = r_des_ee_pose;
            l_curr_ee_pose = l_des_ee_pose;
        }

        // Publish attractors if running in simulation or with fixed values
        publish_task_frames(r_curr_ee_pose, l_curr_ee_pose, right_final_target, left_final_target, task_frame);

        // Current progress variable (position/orientation error).
        r_pos_err = (right_final_target.getOrigin() - r_curr_ee_pose.getOrigin()).length();
        l_pos_err = (left_final_target.getOrigin()  - l_curr_ee_pose.getOrigin()).length();

        //Real Orientation Error qdiff = acos(dot(q1_norm,q2_norm))*180/pi
        r_ori_err = acos(abs(right_final_target.getRotation().dot(r_curr_ee_pose.getRotation())));
        l_ori_err = acos(abs(left_final_target.getRotation().dot(l_ee_pose.getRotation())));

        if (bDisplayDebugInfo){
            ROS_INFO_STREAM_THROTTLE(0.5,"Position Threshold : "    << reachingThreshold    << " ... Current Right Error: " << r_pos_err << " Left Error: " << l_pos_err);
            ROS_INFO_STREAM_THROTTLE(0.5,"Orientation Threshold : " << orientationThreshold << " ... Current Right Error: " << r_ori_err << " Left Error: " << l_ori_err);
         }

        //  >>> Cartesian Trajectory Computation <<<

        // Compute Next Desired EE Pose for Right Arm
        right_cdsRun->setCurrentEEPose(toMatrix4(r_mNextRobotEEPose));
        toPose(right_cdsRun->getNextEEPose(), r_mNextRobotEEPose);
        r_des_ee_pose = r_mNextRobotEEPose;

        // Compute Next Desired EE Pose for Left Arm
        left_cdsRun->setCurrentEEPose(toMatrix4(l_mNextRobotEEPose));
        toPose(left_cdsRun->getNextEEPose(), l_mNextRobotEEPose);

        // Transformation for PHASE_REACH_TO_PEEL Model

        if (bAdditionalTransforms)
            apply_task_specific_transformations(left_final_target, l_mNextRobotEEPose);
        else{
            l_des_ee_pose = l_mNextRobotEEPose;
            l_des_ee_pose.setRotation(l_curr_ee_pose.getRotation().slerp(left_final_target.getRotation(), 0.25) );
        }


        // >>> Force computation <<<
        if (bUseForce_l_arm)
            l_des_ee_pose = update_ee_pose_based_on_force(L_ARM_ID, force_control_axis);

        if (bUseForce_r_arm)
            r_des_ee_pose = update_ee_pose_based_on_force(R_ARM_ID, force_control_axis);

        // Make next pose the current pose for open-loop simulation
        if (just_visualize==true)
            initial_config=false;

        //******************************//
        //  Send the computed ee poses  //
        //******************************//

        if (just_visualize==false){
            filter_arm_motion(r_des_ee_pose, l_des_ee_pose);         // Filter Commands in Cartesian space
            sendPose(r_des_ee_pose, l_des_ee_pose);
        }

        as_.publishFeedback(feedback_);

        // >>>> Check Convergence in Position
        if(r_pos_err < reachingThreshold && l_pos_err < reachingThreshold){

            ROS_INFO("POSITION DYNAMICS CONVERGED!");

            if (bIgnoreOri){
                sendPose(r_curr_ee_pose, l_curr_ee_pose); // Stop the robot after the position has converged
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

            if (bEndInContact_r_arm){
                if (bWaitForForces_right_arm){
                    ROS_INFO_STREAM("In PHASE " << phase << " >> searching for contact now on arm " << R_ARM_ID);
                    bool x_r_arm = find_object_by_contact(R_ARM_ID, search_axis_r_arm, max_search_distance_r_arm, max_vertical_speed_r_arm, max_contact_force_r_arm);
                    return x_r_arm;
                }
                ROS_INFO("Finished Finding Object LOOP");
                break;
            }

            // >>>> Check Convergence in Orientation
            else if((r_ori_err < orientationThreshold) || isnan(r_ori_err)) {
                ROS_INFO("RIGHT ORIENTATION DYN CONVERGED!");
                if((l_ori_err < orientationThreshold) || isnan(l_ori_err)){
                    ROS_INFO("LEFT ORIENTATION DYN CONVERGED!");
                    sendPose(r_curr_ee_pose, l_curr_ee_pose); // Stop the robot by sending the current position again, which results in zero velocity
                    break;
                }
            }
        }

        loop_rate.sleep();
    }
    ROS_INFO("OUT OF LOOP CDS");

    delete right_cdsRun;
    delete left_cdsRun;

    delete r_cdd_cart_filter;
    delete l_cdd_cart_filter;

}


