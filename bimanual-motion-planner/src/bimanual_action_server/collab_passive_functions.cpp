#include "bimanual_action_server.h"

bool BimanualActionServer::collab_passive_model_execution(TaskPhase phase, tf::Transform task_frame, tf::Transform right_att, double dt, CDSController::DynamicsType r_masterType, CDSController::DynamicsType r_slaveType, double reachingThreshold, double orientationThreshold, tf::Transform left_att){


    ROS_INFO_STREAM(" Starting Collaborative Execution, robot master");
    ROS_INFO_STREAM(" Model Path "                  << model_base_path);
    ROS_INFO_STREAM(" Execute Learned model: phase "<< phase);
    ROS_INFO_STREAM(" Reaching threshold "          << reachingThreshold);
    ROS_INFO_STREAM(" Orientation threshold "       << orientationThreshold);
    ROS_INFO_STREAM(" Model DT "                    << model_dt);

    tf::Transform  right_final_target;

    // =======================================================================
    // Here we only control the Right arm acting as master
    // The left robot arm remains stationary
    // The human performs the role of the left arm
    // =======================================================================



    bEnableStiffModel_r_arm = false;
    if (bEnableStiffModel_r_arm){
        initialize_stiffness_model(model_base_path, phase, R_ARM_ID, R_ARM_ROLE);
    }

//    rosrun tf tf_echo  /Vision_Frame/base_link /world
//    - Translation: [0.489, 16.377, -1.011]
//    - Rotation: in Quaternion [0.001, -0.000, -0.000, 1.000]
//                in RPY (radian) [0.001, -0.000, -0.000]
//                in RPY (degree) [0.072, -0.026, -0.004]


//    tf::Pose vision_displacement;
//    vision_displacement.setOrigin(tf::Vector3(0,0,0));
//    vision_displacement.setRotation(tf::createQuaternionFromRPY(-1.57, 0, 0));
//    bowl_in_base_transform.mult(vision_displacement, bowl_in_base_transform);

    //bEnableVision = false;
    // Compute target
    if (bEnableVision){
        ROS_INFO_STREAM("========= Using Frames from vision =============");
        right_final_target.mult(bowl_in_base_transform, right_att);
    }
    else
        right_final_target.mult(task_frame, right_att);



    ROS_INFO_STREAM("Right Target " << right_final_target.getOrigin().x() << " " << right_final_target.getOrigin().y() << " " << right_final_target.getOrigin().z());
    ROS_INFO_STREAM("Human ATT in obj " << left_att.getOrigin().x() << " " << left_att.getOrigin().y() << " " << left_att.getOrigin().z());

    bool bHproximity = false;


    // ======================================================================================================
    // ========= Initialize CDS model of the right arm
    // ======================================================================================================

    // Model Task Frame
    tf::Transform model_task_frame;
    model_task_frame.setIdentity();

    // Setting Initial conditions
    // (For Visualization of Trajectories)
    if (initial_config == true){
        r_curr_ee_pose = r_ee_pose;
        l_curr_ee_pose = h_wrist_pose; // Vision tracking of the human wrist as the left arm
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

    // Variables for execution
    ros::Duration loop_rate(model_dt);
    tf::Pose r_mNextRobotEEPose = r_curr_ee_pose;

    // ======================================================================================================
    // ========= Real time loop
    // ======================================================================================================

    ROS_INFO("Execution started");
    while(ros::ok()) {

        if (initial_config == true){
            // Setting Initial conditions
            r_curr_ee_pose = r_ee_pose;
            l_curr_ee_pose = wrist_in_base_transform;

            // Compensating for the trajectory modifications due to the desired force
            if (bUseForce_r_arm)
                r_curr_ee_pose = remove_correction_due_to_force_from_trajectory(R_ARM_ID, force_control_axis);
        }
        else{
            // For visualization of trajectories
            r_curr_ee_pose = r_des_ee_pose;
            l_curr_ee_pose = wrist_in_base_transform;
        }

        // Publish attractors if running in simulation or with fixed values
       // publish_task_frames(r_curr_ee_pose, h_wrist_pose, right_final_target, left_final_target, task_frame);

        // Current progress variable (position/orientation error).
        r_pos_err = (right_final_target.getOrigin() - r_curr_ee_pose.getOrigin()).length();

        //Real Orientation Error qdiff = acos(dot(q1_norm,q2_norm))*180/pi
        r_ori_err = acos(abs(right_final_target.getRotation().dot(r_curr_ee_pose.getRotation())));

        if (bDisplayDebugInfo){
            ROS_INFO_STREAM_THROTTLE(0.5,"Position Threshold : "    << reachingThreshold    << " ... Current Right Error: " << r_pos_err);
            ROS_INFO_STREAM_THROTTLE(0.5,"Orientation Threshold : " << orientationThreshold << " ... Current Right Error: " << r_ori_err);
         }

        //  >>> Cartesian Trajectory Computation <<<

        // Update attractor frame
        if (bEnableVision){
            right_final_target.mult(bowl_in_base_transform, right_att);
        }
        else
            right_final_target.mult(task_frame, right_att);
        right_cdsRun->setAttractorFrame(toMatrix4(right_final_target));

        // Compute Next Desired EE Pose for Right Arm
        right_cdsRun->setCurrentEEPose(toMatrix4(r_mNextRobotEEPose));
        toPose(right_cdsRun->getNextEEPose(), r_mNextRobotEEPose);
        r_des_ee_pose = r_mNextRobotEEPose;

        // >>> Force computation <<<
        if (bUseForce_r_arm)
            r_des_ee_pose = update_ee_pose_based_on_force(R_ARM_ID, force_control_axis);

        // Make next pose the current pose for open-loop simulation
        if (just_visualize==true)
            initial_config=false;

        // ===========================
        // Track the human


        // Compute transforms from the TF
//        tf::StampedTransform human_final_target;
//        human_final_target.mult(right_final_target, left_att);
//        ROS_INFO_STREAM("Human final target " << human_final_target.getOrigin().x() << " " << human_final_target.getOrigin().y() << " " << human_final_target.getOrigin().z() );
//
//        tf::TransformListener listener;
//        try {
//            listener.waitForTransform("/human_final_target", "/wrist_in_base_transform", ros::Time(0), ros::Duration(10.0) );
//            listener.lookupTransform("/human_final_target", "/wrist_in_base_transform",  ros::Time(0), wrist_in_att_transform);
//        } catch (tf::TransformException ex) {
//            ROS_ERROR("%s",ex.what());
//        }
//        ROS_INFO_STREAM("Wrist in ATT " << wrist_in_att_transform.getOrigin().x() << " " << wrist_in_att_transform.getOrigin().y() << " " << wrist_in_att_transform.getOrigin().z());
//
//        double h_pos_err_d;
//        h_pos_err_d = (human_final_target.getOrigin() - wrist_in_att_transform.getOrigin()).length();
//        h_ori_err = acos(human_final_target.getRotation().dot(wrist_in_att_transform.getRotation()));
//        ROS_INFO_STREAM("human pos err: "  << h_pos_err_d << " human ori err: " << h_ori_err);


        // Distance based tracking (i.e. distance of wrist frame wrt bowl frame same as att frame as the att is set relative to the bowl)

        tf::Vector3 dist_wrist_bowl;
        dist_wrist_bowl = h_wrist_pose.getOrigin() - bowl_frame.getOrigin();

        h_pos_err[0] = abs(dist_wrist_bowl[0])-abs(left_att.getOrigin().x());
        h_pos_err[1] = abs(dist_wrist_bowl[1])-abs(left_att.getOrigin().y());
        h_pos_err[2] = abs(dist_wrist_bowl[2])-abs(left_att.getOrigin().z());
        h_pos_err = h_pos_err.absolute();

        double h_pos_thr_x = 0.10;
        double h_pos_thr_y = 0.10;
        double h_pos_thr_z = 0.15;

        if (h_pos_err[0] <= h_pos_thr_x || h_pos_err[1] <= h_pos_thr_y || h_pos_err[2] <= h_pos_thr_z){
            bHproximity = true;
            ROS_INFO_STREAM("Human Converged");
        }

//        ROS_INFO_STREAM("Human Reaching Err " << " X: " << h_pos_err[0] << " Y: " << h_pos_err[1]  << " Z: " <<  h_pos_err[2]);




        //******************************//
        //  Send the computed ee poses  //
        //******************************//


        if (just_visualize==false){
            filter_arm_motion(r_des_ee_pose, h_wrist_pose);         // Filter Commands in Cartesian space
            sendPoseRight(r_des_ee_pose);
            if (bHproximity){
                sendJStiffCmd(TASK_STIFFNESS, R_ARM_ID);
            }
            else
                sendJStiffCmd(INTERACTION_STIFFNESS, R_ARM_ID);

            if (bEnableStiffModel_r_arm){
                double des_avg_stiff;
                mStiffModel_r_arm -> getGMROutput(&r_pos_err, &des_avg_stiff);
                // TODO: Convert cartesian stiffness to joint stiffness properly.
                sendJStiffCmd(des_avg_stiff, R_ARM_ID);
            }
        }

        as_.publishFeedback(feedback_);

        // >>>> Check Convergence in Position >> and check the state of the human
            if(r_pos_err < reachingThreshold && h_current_action_state == true ){

            ROS_INFO("POSITION DYNAMICS CONVERGED!");

            if (bIgnoreOri){
                sendPoseRight(r_curr_ee_pose); // Stop the robot after the position has converged
                break;
            }

            // >>>> Check if contact should be established at the end of the action

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
                sendPoseRight(r_curr_ee_pose); // Stop the robot by sending the current position again, which results in zero velocity
                sendJStiffCmd(r_avg_jstiff, R_ARM_ID);
                break;
            }
        }

        loop_rate.sleep();
    }
    ROS_INFO("OUT OF LOOP CDS");

    delete right_cdsRun;

    delete r_cdd_cart_filter;



}


