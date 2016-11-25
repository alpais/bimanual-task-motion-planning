#include "bimanual_action_server.h"

bool BimanualActionServer::collab_passive_model_execution(TaskPhase phase, tf::Transform task_frame, tf::Transform right_att, double dt, CDSController::DynamicsType r_masterType, CDSController::DynamicsType r_slaveType, double reachingThreshold, double orientationThreshold, tf::Transform left_att){


    ROS_INFO_STREAM(" Starting Collaborative Execution, robot master");
    ROS_INFO_STREAM(" Model Path "                  << model_base_path);
    ROS_INFO_STREAM(" Execute Learned model: phase "<< phase);
    ROS_INFO_STREAM(" Reaching threshold "          << reachingThreshold);
    ROS_INFO_STREAM(" Orientation threshold "       << orientationThreshold);
    ROS_INFO_STREAM(" Model DT "                    << model_dt);

    // =======================================================================
    // Here we only control the Right arm acting as master
    // The left robot arm remains stationary
    // The human performs the role of the left arm
    // =======================================================================

    tf::Transform  right_final_target, human_estimated_target;

    // ------- computing the displacement of the vision world frame with respect to the robot base frame

    tf::Pose world_frame;   world_frame.setIdentity();
    tf::Pose virtual_world; virtual_world.setOrigin(tf::Vector3(0,0,0));                virtual_world.setRotation(tf::Quaternion(0.000, 0.000, 0.707, 0.707));
    tf::Pose vision_world;  vision_world.setOrigin(tf::Vector3(0.482, 16.376, -1.028)); vision_world.setRotation(tf::Quaternion(0.000, 0.001, 0.000, 1.000));
    tf::Pose vision_displacement; vision_displacement.mult(virtual_world, vision_world);

    // -------- >> Compute the initial target
    if (bEnableVision){
        ROS_INFO_STREAM("========= Using Frames from vision =============");
        right_final_target.mult(bowl_in_base_transform, right_att); // here using the transform computed once in the action server, updating automatically in the RT loop later
        human_estimated_target.mult(r_curr_ee_pose, left_att);
    }
    else
        right_final_target.mult(task_frame, right_att);

    ROS_INFO_STREAM("Right Target " << right_final_target.getOrigin().x() << " " << right_final_target.getOrigin().y() << " " << right_final_target.getOrigin().z());


    // ======================================================================================================
    // ========= Initializing models and stuff
    // ======================================================================================================

    // ------- >> Stiffness
    bEnableStiffModel_r_arm = false;
    if (bEnableStiffModel_r_arm){
        initialize_stiffness_model(model_base_path, phase, R_ARM_ID, R_ARM_ROLE);
    }

    // ------- >> CDS for the master arm
    // here the master moves relative to the bowl
    // no arm coupling as we cannot control the slave

    // Model Task Frame
    tf::Transform model_task_frame;
    model_task_frame.setIdentity();

    // Setting Initial conditions
    // (For Visualization of Trajectories)
    if (initial_config == true){
        r_curr_ee_pose = r_ee_pose;
        l_curr_ee_pose = wrist_in_base_transform; // >>>>>> NOT UPDATED NOW Vision tracking of the human wrist as the left arm
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

    bool bHproximity = false;


    // ----- >> Human arm

    read_grasp_specification(phase, model_base_path, L_ARM_ROLE, L_ARM_ID);

    // get total active joints in this action
    int total_active_joints = 0;
    MathLib::Vector active_joints_idx;
    for (int i = 0; i < nFingerJoints; i ++){
        if (finger_joints_mask(i) == 1)
            total_active_joints ++;
    }

    // get indexes of active joints
    active_joints_idx.Resize(total_active_joints);
    int j = 0;
    for (int i = 0; i < nFingerJoints; i ++){
        if (finger_joints_mask(i) == 1){
            active_joints_idx(j) = i;
            j++;
        }
    }

    // printing values
    for (int i = 0; i<total_active_joints; i++)
        ROS_INFO_STREAM("Using Joint : " << active_joints_idx(i)+1  << " avg " << finger_joints_avg(active_joints_idx(i)) << " delta " << finger_joints_deltas(active_joints_idx(i)));

    // initialize grasp_shape -> GQ model

    // initialize GQ -> expected force model


    // ======================================================================================================
    // ========= Real time loop
    // ======================================================================================================

    ROS_INFO("Execution started");
    while(ros::ok()) {

        // ------- >> Updating targets from vision for both the human and the robot

        tf::Pose wrist_in_robot_base; wrist_in_robot_base.mult(vision_displacement, vision_wrist_frame);
        human_estimated_target.mult(r_curr_ee_pose, left_att);

        tf::Pose bowl_in_robot_base; bowl_in_robot_base.mult(vision_displacement, vision_bowl_frame);
        right_final_target.mult(bowl_in_robot_base, right_att);

        // ------ >> Updating state
        if (initial_config == true){
            // Setting Initial conditions
            r_curr_ee_pose = r_ee_pose;
            l_curr_ee_pose = wrist_in_robot_base;

            // Compensating for the trajectory modifications due to the desired force
            if (bUseForce_r_arm)
                r_curr_ee_pose = remove_correction_due_to_force_from_trajectory(R_ARM_ID, force_control_axis);
        }
        else{
            // For visualization of trajectories
            r_curr_ee_pose = r_des_ee_pose;
            l_curr_ee_pose = wrist_in_robot_base;
        }

        // ------- >> Publishing attractors if running in simulation or with fixed values
        publish_task_frames(r_curr_ee_pose, wrist_in_robot_base, right_final_target, human_estimated_target, task_frame);

        r_pos_err = (right_final_target.getOrigin() - r_curr_ee_pose.getOrigin()).length();         // Current progress variable (position/orientation error)
        r_ori_err = acos(abs(right_final_target.getRotation().dot(r_curr_ee_pose.getRotation())));  //Real Orientation Error qdiff = acos(dot(q1_norm,q2_norm))*180/pi

        if (bDisplayDebugInfo){
            ROS_INFO_STREAM_THROTTLE(0.5,"Position Threshold : "    << reachingThreshold    << " ... Current Right Error: " << r_pos_err);
            ROS_INFO_STREAM_THROTTLE(0.5,"Orientation Threshold : " << orientationThreshold << " ... Current Right Error: " << r_ori_err);
        }

        // ------- >> Computing the cartesian trajectory

        right_cdsRun->setAttractorFrame(toMatrix4(right_final_target));
        right_cdsRun->setCurrentEEPose(toMatrix4(r_mNextRobotEEPose));
        toPose(right_cdsRun->getNextEEPose(), r_mNextRobotEEPose);
        r_des_ee_pose = r_mNextRobotEEPose;

        // ------- >> Computing the force to be applied
        if (bUseForce_r_arm)
            r_des_ee_pose = update_ee_pose_based_on_force(R_ARM_ID, force_control_axis);

        // ------- >> Make next pose the current pose for open-loop simulation
        if (just_visualize==true)
            initial_config=false;

        // ------- >> Estimate human state
        double intent_modulation; intent_modulation = 1;

        // don't really care about the orientation
        h_pos_err = human_estimated_target.getOrigin().absolute() -  wrist_in_robot_base.getOrigin().absolute();
        h_pos_err = h_pos_err.absolute();

        double h_pos_thr_x = 0.10;
        double h_pos_thr_y = 0.10;
        double h_pos_thr_z = 0.15;

        //        ROS_INFO_STREAM("h proximity " << " X: " << h_pos_err[0] << " Y: " << h_pos_err[1]  << " Z: " <<  h_pos_err[2]);

        h_pub_crt_dist_err(h_pos_err);

        if (h_pos_err[0] <= h_pos_thr_x && h_pos_err[1] <= h_pos_thr_y && h_pos_err[2] <= h_pos_thr_z){
            bHproximity = true;
            intent_modulation = 8;
        }
        else
            bHproximity = false;

        // Check finger joint configuration
        for (int i=0; i<total_active_joints; i++){ // for each joint
            if ((finger_joints_all(active_joints_idx(i)) < finger_joints_avg(active_joints_idx(i)) + finger_joints_deltas(active_joints_idx(i))) &&
                    (finger_joints_all(active_joints_idx(i)) > finger_joints_avg(active_joints_idx(i)) - finger_joints_deltas(active_joints_idx(i))))
                // check that the values are withing the avergae +/- deltas
                ROS_INFO_STREAM("Joint " << active_joints_idx(i)+1 << " good Configuration " << finger_joints_all(i));
            else
                ROS_INFO_STREAM("Joint " << active_joints_idx(i)+1 << " bad configuration");
        }

        if (bGloveTekscanInitialized){

            ROS_INFO_STREAM("Pressure: Thumb " << avg_thumb_pressure << " Index " << avg_index_pressure << " Palm " << avg_palm_pressure);
            ROS_INFO_STREAM("JA: Thumb " << thumb_ja(0) << " Index " << index_ja(0) << " "  << index_ja(1) << " "  << index_ja(2) << " ");
        }


        // ------- >> Update stiffness
        bool bEnableInteractionStiffnessModulation = true;   // true if the robot should update its stiffness based
        // on the estimated intention of the human to apply a force
        // Used only in reaching-type of actions

        double des_avg_stiff, model_stiff_modulation;
        des_avg_stiff = INTERACTION_STIFFNESS;

        if (bUseForce_l_arm){ // >> the human arm is expected to apply forces in the current action
            if (bEnableStiffModel_r_arm){
                mStiffModel_r_arm -> getGMROutput(&r_pos_err, &model_stiff_modulation);
                des_avg_stiff = TASK_STIFFNESS * model_stiff_modulation;
            }
            else des_avg_stiff = TASK_STIFFNESS;
        }
        else {
            if (bEnableInteractionStiffnessModulation){
                des_avg_stiff = INTERACTION_STIFFNESS * intent_modulation;
            }
            else
                des_avg_stiff = INTERACTION_STIFFNESS;
        }

        // ROS_INFO_STREAM("Current Stiffness " << des_avg_stiff);

        // TODO: Convert cartesian stiffness to joint stiffness properly.

        //******************************//
        //  Send the computed ee pose  //
        //******************************//

        if (just_visualize==false){
            filter_arm_motion(r_des_ee_pose, wrist_in_robot_base);         // Filter Commands in Cartesian space
            sendPoseRight(r_des_ee_pose);
            sendJStiffCmd(des_avg_stiff, R_ARM_ID);
        }

        as_.publishFeedback(feedback_);

        // ---- >> Check Convergence in Position and the state of the human
        if(r_pos_err < reachingThreshold && h_current_action_state == true ){

            ROS_INFO("POSITION DYNAMICS CONVERGED!");

            if (bIgnoreOri){
                sendPoseRight(r_curr_ee_pose); // Stop the robot after the position has converged
                break;
            }

            // ----- >> Check if contact should be established at the end of the action
            if (bEndInContact_r_arm){
                if (bWaitForForces_right_arm){
                    ROS_INFO_STREAM("In PHASE " << phase << " >> searching for contact now on arm " << R_ARM_ID);
                    bool x_r_arm = find_object_by_contact(R_ARM_ID, search_axis_r_arm, max_search_distance_r_arm, max_vertical_speed_r_arm, max_contact_force_r_arm);
                    return x_r_arm;
                }
                ROS_INFO("Finished Finding Object LOOP");
                break;
            }

            // ----- >> Check Convergence in Orientation
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


// NOTE: Can put the listerner in the RT loop, but waiting for the transform makes it all too slow
// Better to read the poses from ros topics and compute all the transformations internally
/*
tf::TransformListener listener;
try {
    listener.waitForTransform("/world_frame", "Bowl_Frame/base_link", ros::Time(0), ros::Duration(1.0) );
    listener.lookupTransform("/world_frame", "Bowl_Frame/base_link",  ros::Time(0), bowl_in_base_transform);
} catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
}

if (bEnableVision){
    right_final_target.mult(bowl_in_base_transform, right_att);
}
else
    right_final_target.mult(task_frame, right_att);
 */

