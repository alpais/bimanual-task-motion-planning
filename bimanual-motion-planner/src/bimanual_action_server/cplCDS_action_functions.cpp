#include "bimanual_action_server.h"

bool BimanualActionServer::coupled_learned_model_execution(TaskPhase phase, CDSController::DynamicsType masterType,
                                                           CDSController::DynamicsType slaveType, double reachingThreshold,
                                                           double orientationThreshold, tf::Transform task_frame,
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
    } else if (task_id == SCOOPING_TASK_ID && (phase == PHASE_SCOOP_DEPART || phase == PHASE_SCOOP_TRASH)){
        right_final_target.mult(task_frame, right_att);
    }
    else
        right_final_target = r_ee_pose;

    // In the Scooping task the Master Arm maintains its position just in the scooping phase
    // In depart it goes away few cm >> see Att in the python script
    // In Trash it goes away even more

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

            fixed_reach_to_peel_attr.setOrigin(tf::Vector3(-0.130, 0.073, 0.238));
            fixed_reach_to_peel_attr.setRotation(tf::Quaternion(-0.333, 0.753, -0.453, 0.342));
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
        fixed_right_arm_rf.setRotation(tf::Quaternion(0.8821, 0.3088, 0.0049, 0.3557));
        fixed_right_arm_rf.setOrigin(r_ee_pose.getOrigin());
        // To determine ATT run  >> rosrun tf tf_echo /TOOL_ft /Hand_ft
        if (phase == PHASE_SCOOP_REACH_TO_SCOOP){
            tf::Transform fixed_reach_to_scoop_att;
            fixed_reach_to_scoop_att.setOrigin(tf::Vector3(0.117, 0.079, 0.232)); // from tf echo
            fixed_reach_to_scoop_att.setRotation(tf::Quaternion(0.8925, -0.1301, 0.2192, 0.3720)); // Tf transform * Rx(pi/2)
            left_final_target.mult(fixed_right_arm_rf, fixed_reach_to_scoop_att);
        } else if (phase == PHASE_SCOOP_SCOOP){
            tf::Transform fixed_scoop_att;
            fixed_scoop_att.setOrigin(tf::Vector3(0.027, -0.043, 0.241));
            fixed_scoop_att.setRotation(tf::Quaternion(0.8925, -0.1301, 0.2192, 0.3720));
            left_final_target.mult(fixed_right_arm_rf, fixed_scoop_att);
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
    right_cdsRun->setMotionParameters(0.5,1,1,reachingThreshold, masterType, slaveType);
    right_cdsRun->postInit();


    // Initialize CDS for left arm
    CDSExecution *left_cdsRun = new CDSExecution;
    left_cdsRun->initSimple(model_base_path, phase, L_ARM_ID, L_ARM_ROLE);

    // -> Apply Rotation (pi on Y in Origin RF)
    // The following two lines are correct only if the left arm moves wrt the right arm
    left_cdsRun->setObjectFrame(toMatrix4(model_task_frame));
    left_cdsRun->setAttractorFrame(toMatrix4(left_final_target));

    // otherwise use the following initialization wrt the task frame
    //    left_cdsRun->setObjectFrame(toMatrix4(task_frame));
    //    left_cdsRun->setAttractorFrame(toMatrix4(left_att));

    left_cdsRun->setCurrentEEPose(toMatrix4(l_curr_ee_pose));
    left_cdsRun->setDT(model_dt);
    left_cdsRun->setMotionParameters(1,1,1,reachingThreshold, masterType, slaveType);
    left_cdsRun->postInit();


    // Variable for execution
    ros::Duration loop_rate(model_dt);
    tf::Pose r_mNextRobotEEPose = r_curr_ee_pose;
    tf::Pose l_mNextRobotEEPose = l_curr_ee_pose;

    double r_pos_err, r_ori_err, l_pos_err, l_ori_err;

    double z_force_correction = 0;
    double z_desired_force = 8;
    double z_force_correction_max = 0.05; // 1cm
    double z_force_correction_delta = 0.001; // 1 mm

    // ======================================================================================================
    // ========= Real time loop
    // ======================================================================================================

    ROS_INFO("Execution started");
    while(ros::ok()) {

        // Setting Initial conditions
        if (initial_config == true){
            r_curr_ee_pose = r_ee_pose;
            l_curr_ee_pose = l_ee_pose;
            if (task_id == PEELING_TASK_ID && phase == PHASE_PEEL) {
                // trick the CDS to think there's no correction
                tf::Vector3& origin = l_curr_ee_pose.getOrigin();
                origin[2] += z_force_correction;
                l_curr_ee_pose.setOrigin(origin);
            }
        }
        else{ // For visualization of trajectories
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
        l_ori_err = acos(abs(left_final_target.getRotation().dot(l_curr_ee_pose.getRotation())));
        ROS_INFO_STREAM_THROTTLE(0.5,"Position Threshold : "    << reachingThreshold    << " ... Current Right Error: " << r_pos_err << " Left Error: " << l_pos_err);
        ROS_INFO_STREAM_THROTTLE(0.5,"Orientation Threshold : " << orientationThreshold << " ... Current Right Error: " << r_ori_err << " Left Error: " << l_ori_err);


        // Compute Next Desired EE Pose for Right Arm
        right_cdsRun->setCurrentEEPose(toMatrix4(r_mNextRobotEEPose));
        toPose(right_cdsRun->getNextEEPose(), r_mNextRobotEEPose);
        r_des_ee_pose = r_mNextRobotEEPose;

        // Compute Next Desired EE Pose for Left Arm
        left_cdsRun->setCurrentEEPose(toMatrix4(l_mNextRobotEEPose));
        toPose(left_cdsRun->getNextEEPose(), l_mNextRobotEEPose);

        // Transformation for PHASE_REACH_TO_PEEL Model
        if ((task_id == PEELING_TASK_ID && phase == PHASE_REACH_TO_PEEL) || (task_id = SCOOPING_TASK_ID && phase == PHASE_SCOOP_REACH_TO_SCOOP)){
            tf::Transform  l_ee_rot, ee_2_rob;
            l_ee_rot.setIdentity(); ee_2_rob.setIdentity();

            // Transform Attractor to Origin
            l_des_ee_pose.mult(left_final_target.inverse(),l_mNextRobotEEPose);

            // -> Apply Rotation (pi on Z in Origin RF)
            l_ee_rot.setBasis(tf::Matrix3x3(-1,0,0,0,-1,0,0,0,1)); //z (pi)
            l_des_ee_pose.mult(l_ee_rot,l_des_ee_pose);

            // -> Apply Rotation (pi on X in Origin RF)
            //l_ee_rot.setBasis(tf::Matrix3x3(1,0,0,0,cos,-sin,0,sin,cos)); //x
            l_ee_rot.setBasis(tf::Matrix3x3(1,0,0,0,0,1,0,-1,0)); //x (-pi/2)
            l_des_ee_pose.mult(l_ee_rot,l_des_ee_pose);

            // -> Transform back to Robot
            l_des_ee_pose.mult(left_final_target,l_des_ee_pose);
            l_des_ee_pose.setRotation(l_curr_ee_pose.getRotation().slerp(left_final_target.getRotation(), 0.75) );

            // Don't Care about master
            if (r_pos_err > 0.02)
                r_des_ee_pose = r_curr_ee_pose;
        }
        else
            l_des_ee_pose = l_mNextRobotEEPose;


        if (task_id == PEELING_TASK_ID && phase == PHASE_PEEL){
            // TODO Check the force first

            Eigen::VectorXd ee_ft;
            ee_ft.resize(6);
            ee_ft = l_curr_ee_ft;

            double z_crt_force = ee_ft[2];
            if (z_crt_force < z_desired_force && z_force_correction <= z_force_correction_max){
                z_force_correction += z_force_correction_delta;
            }
            //            l_des_ee_pose.setOrigin().z(z_force_correction);
            tf::Vector3& origin = l_des_ee_pose.getOrigin();
            origin[2] -= z_force_correction;
            l_des_ee_pose.setOrigin(origin);
        }

        // Make next pose the current pose for open-loop simulation
        if (just_visualize==true)
            initial_config=false;

        //******************************//
        //  Send the computed ee poses  //
        //******************************//
        if (just_visualize==false)
            sendPose(r_des_ee_pose, l_des_ee_pose);

        as_.publishFeedback(feedback_);

        if(r_pos_err < reachingThreshold && l_pos_err < reachingThreshold){

            ROS_INFO_STREAM("POSITION DYNAMICS CONVERGED!");

            if (phase == PHASE_REACH_TO_PEEL){
                ROS_INFO_STREAM("In PHASE_REACH_TO_PEEL.. finding zucchini now...");
                //sendPose(r_curr_ee_pose, l_curr_ee_pose);
                if (bWaitForForces_left_arm)	{
                    bool x_l_arm = find_object_by_contact(L_ARM_ID, 0.07, 0.01, 8);
                    return x_l_arm;
                }
                ROS_INFO("Finished Finding Object LOOP");
                break;
            }

            if ((task_id == PEELING_TASK_ID && phase == PHASE_PEEL) || (task_id == SCOOPING_TASK_ID && phase == PHASE_SCOOP_SCOOP) || (task_id == SCOOPING_TASK_ID && phase == PHASE_SCOOP_REACH_TO_SCOOP)){
                sendPose(r_curr_ee_pose, l_curr_ee_pose);
                break;
            }

            else if((r_ori_err < orientationThreshold) || isnan(r_ori_err)) {
                ROS_INFO_STREAM("RIGHT ORIENTATION DYN CONVERGED!");
                if((l_ori_err < orientationThreshold) || isnan(l_ori_err)){
                    ROS_INFO_STREAM("LEFT ORIENTATION DYN CONVERGED!");
                    sendPose(r_curr_ee_pose, l_curr_ee_pose);
                    break;
                }
            }
            //            break;
        }

        ROS_INFO("SLEEEPING NOW");
        loop_rate.sleep();
    }
    ROS_INFO("OUT OF LOOP CDS");
    delete right_cdsRun;
    delete left_cdsRun;

}


