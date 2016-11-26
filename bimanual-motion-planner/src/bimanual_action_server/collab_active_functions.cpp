#include "bimanual_action_server.h"

bool BimanualActionServer::collab_active_model_execution(TaskPhase phase, tf::Transform task_frame, tf::Transform left_att, double dt, CDSController::DynamicsType l_masterType, CDSController::DynamicsType l_slaveType, double reachingThreshold, double orientationThreshold, tf::Transform right_att){

    ROS_INFO_STREAM(" Starting Collaborative Execution, human master");
    ROS_INFO_STREAM(" Model Path "                  << model_base_path);
    ROS_INFO_STREAM(" Execute Learned model: phase "<< phase);
    ROS_INFO_STREAM(" Reaching threshold "          << reachingThreshold);
    ROS_INFO_STREAM(" Orientation threshold "       << orientationThreshold);
    ROS_INFO_STREAM(" Model DT "                    << model_dt);

    // =========================================================
    // Left Arm moves relative to the master arm
    // Get Right arm position from vision (i.e. the wrist)
    // assume the human arm positions conveniently wrt the bowl
    // =========================================================

    tf::Transform  left_final_target, human_estimated_target; human_estimated_target.setIdentity();

    // ------- computing the displacement of the vision world frame with respect to the robot base frame

    tf::Pose world_frame;   world_frame.setIdentity();
    tf::Pose virtual_world; virtual_world.setOrigin(tf::Vector3(0,0,0));                virtual_world.setRotation(tf::Quaternion(0.000, 0.000, 0.707, 0.707));
    tf::Pose vision_world;  vision_world.setOrigin(tf::Vector3(0.482, 16.376, -1.028)); vision_world.setRotation(tf::Quaternion(0.000, 0.001, 0.000, 1.000));
    tf::Pose vision_displacement; vision_displacement.mult(virtual_world, vision_world);


    // -------- >> Compute the initial target
    if (bEnableVision){
        ROS_INFO_STREAM("========= Using Frames from vision =============");
        left_final_target.mult(wrist_in_base_transform, left_att); // here using the transform computed once in the action server, updating automatically in the RT loop later
        human_estimated_target.mult(bowl_in_base_transform, right_att);
    }
    else
        left_final_target.mult(task_frame, left_att); // the left ATT is given the master RF

    ROS_INFO_STREAM("Left Target " << left_final_target.getOrigin().x() << " " << left_final_target.getOrigin().y() << " " << left_final_target.getOrigin().z());

    tf::Transform wrist_in_robot_base; wrist_in_robot_base.setIdentity(); // >> UPDATE FROM VISION

    // ======================================================================================================
    // ========= Initialize models
    // ======================================================================================================


    // ------- >> CDS for the master arm
    // here the robot acts as slave and moves wrt the human arm
    // coupling is used to adapt to the master's motion

    // Model Task Frame
    tf::Transform model_task_frame;
    model_task_frame.setIdentity();

    // Setting Initial conditions
    // (For Visualization of Trajectories)
    if (initial_config == true){
        r_curr_ee_pose = wrist_in_base_transform; // >>> COMPUTE IT IN ROB FRAME
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


    bool bHproximity = false;


    // ======================================================================================================
    // ========= Real time loop
    // ======================================================================================================

    ROS_INFO("Execution started");
    while(ros::ok()) {

        // ------- >> Updating targets from vision for both the human and the robot

        tf::Pose bowl_in_robot_base; bowl_in_robot_base.mult(vision_displacement, vision_bowl_frame);
        tf::Pose wrist_in_robot_base; wrist_in_robot_base.mult(vision_displacement, vision_wrist_frame);

        human_estimated_target.mult(bowl_in_robot_base, right_att);

        if (bHproximity){
            tf::Transform fixed_right_arm_rf;
            fixed_right_arm_rf.setIdentity();
            fixed_right_arm_rf.setRotation(tf::Quaternion(wrist_in_robot_base.getRotation()));
            fixed_right_arm_rf.setOrigin(wrist_in_robot_base.getOrigin());
            left_final_target.mult(fixed_right_arm_rf, left_att);
        }
        else
            left_final_target.mult(wrist_in_robot_base, left_att);

        // ------ >> Updating state

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

        // ------- >> Publishing attractors if running in simulation or with fixed values
        publish_task_frames(r_curr_ee_pose, l_curr_ee_pose, human_estimated_target, left_final_target, bowl_in_robot_base);

        // Current progress variable (position/orientation error).
        double h_pos_err_d;
        h_pos_err_d = (human_estimated_target.getOrigin() - wrist_in_robot_base.getOrigin()).length();
        l_pos_err = (left_final_target.getOrigin()  - l_curr_ee_pose.getOrigin()).length();

        //Real Orientation Error qdiff = acos(dot(q1_norm,q2_norm))*180/pi
        h_ori_err = acos(abs(human_estimated_target.getRotation().dot(wrist_in_robot_base.getRotation())));
        l_ori_err = acos(abs(left_final_target.getRotation().dot(l_ee_pose.getRotation())));

        if (bDisplayDebugInfo){
            ROS_INFO_STREAM_THROTTLE(0.5,"Position Threshold : "    << reachingThreshold    << " ... Current Right Error: " << h_pos_err_d << " Left Error: " << l_pos_err);
            ROS_INFO_STREAM_THROTTLE(0.5,"Orientation Threshold : " << orientationThreshold << " ... Current Right Error: " << h_ori_err << " Left Error: " << l_ori_err);
        }


        // Update coupling and CDS parameters


        // ------- >> Computing the cartesian trajectory


        left_cdsRun->setAttractorFrame(toMatrix4(left_final_target));
        left_cdsRun->setCurrentEEPose(toMatrix4(l_mNextRobotEEPose));
        toPose(left_cdsRun->getNextEEPose(), l_mNextRobotEEPose);

        l_des_ee_pose = l_mNextRobotEEPose;
        l_des_ee_pose.setRotation(l_curr_ee_pose.getRotation().slerp(left_final_target.getRotation(), 0.25) );


        // ------- >> Computing the force to be applied
        if (bUseForce_l_arm)
            l_des_ee_pose = update_ee_pose_based_on_force(L_ARM_ID, force_control_axis);

        // ------- >> Make next pose the current pose for open-loop simulation
        if (just_visualize==true)
            initial_config=false;

        if (bActionTypeReach){

            double intent_modulation; intent_modulation = 1;

            // distance to estimated attractor
            h_pos_err = human_estimated_target.getOrigin().absolute() -  wrist_in_robot_base.getOrigin().absolute();
            h_pos_err = h_pos_err.absolute();


            if (bDisplayDebugInfo)
                ROS_INFO_STREAM("h proximity " << " X: " << h_pos_err[0] << " Y: " << h_pos_err[1]  << " Z: " <<  h_pos_err[2]);

            h_pub_crt_dist_err(h_pos_err);

            if (h_pos_err[0] <= h_pos_thr_x && h_pos_err[1] <= h_pos_thr_y && h_pos_err[2] <= h_pos_thr_z){
                bHproximity = true;
                intent_modulation = 4;
                ROS_INFO_STREAM("Stiffness increase due to proximity");
            }
            else
                bHproximity = false;

//            if (bGloveTekscanInitialized){

//                // Check the hand shape

//                int joints_in_good_config = 0;

//                for (int i=0; i<total_active_joints; i++){ // for each joint
//                    if ((finger_joints_all(active_joints_idx(i)) < finger_joints_avg(active_joints_idx(i)) + 2*finger_joints_deltas(active_joints_idx(i))) &&
//                            (finger_joints_all(active_joints_idx(i)) > finger_joints_avg(active_joints_idx(i)) - 2*finger_joints_deltas(active_joints_idx(i)))){
//                        // check that the values are withing the avergae +/- deltas
//                        ROS_INFO_STREAM("Joint " << active_joints_idx(i)+1 << " good configuration " << finger_joints_all(i));
//                        joints_in_good_config++;
//                    }
//                    else
//                        ROS_INFO_STREAM("Joint " << active_joints_idx(i)+1 << " bad configuration" << "     (Ideal " << finger_joints_avg(active_joints_idx(i)) << " Current " << finger_joints_all(active_joints_idx(i)) << ")");
//                }

//                ROS_INFO_STREAM("Joints in good config " << joints_in_good_config << " out of " << active_joints_idx.Size());

//                ROS_INFO_STREAM("Pressure: Fingertips: Thumb " << thumb_pressure(0) << " Index " << index_pressure(0) << " Middle " << middle_pressure(0) << " Ring " << ring_pressure(0) << " Pinky " << pinky_pressure(0));


//                if (joints_in_good_config >= floor(active_joints_idx.Size()/2)){
//                    intent_modulation = 8;
//                    ROS_INFO_STREAM("Stiffness increase due to the detected hand shape");
//                }

//                // ------- >> Update stiffness
//                bool bEnableInteractionStiffnessModulation = true;      // true if the robot should update its stiffness based
//                // on the estimated intention of the human to apply a force
//                // Used only in reaching-type of actions

//                des_avg_stiff = INTERACTION_STIFFNESS;

//                if (bEnableInteractionStiffnessModulation){
//                    des_avg_stiff = INTERACTION_STIFFNESS * intent_modulation;
//                }
//                else
//                    des_avg_stiff = INTERACTION_STIFFNESS;

//                ROS_INFO_STREAM("Current Stiffness " << des_avg_stiff);
//            }
        }

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


