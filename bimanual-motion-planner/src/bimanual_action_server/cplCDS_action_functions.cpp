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


    tf::Transform  right_final_target, left_final_target;

    // Master Arm Stays in the Position/Orientation for all but ROTATE/REACH, here a give a fixed rotation value
    // 45 degrees on the Z axis (Should be substituted by vision - no time for this now - subscribed to /zuch/feats)
    if (phase == PHASE_ROTATE){
        right_final_target = r_ee_pose;
        tf::Transform delta_rot; delta_rot.setIdentity();
        right_final_target.mult(r_ee_pose.inverse(),right_final_target);
        delta_rot.setBasis(tf::Matrix3x3(0.707,-0.707,0,0.707,0.707,0,0,0,1)); // Rotate around Z EE RF (45dg)
        right_final_target.mult(delta_rot,right_final_target);
        right_final_target.mult(r_ee_pose,right_final_target);
    }
    else
        right_final_target = r_ee_pose;


    // Slave Arm has Attractor on Object relative to a Right Hand Fixed RF
    // (SHould be substituted by vision (i.e. compute attractors from point cloud) - no time for this now)
    if (phase == PHASE_REACH_TO_PEEL || phase == PHASE_PEEL){
        tf::Transform fixed_right_arm_rf;
        fixed_right_arm_rf.setIdentity();
        fixed_right_arm_rf.setRotation(tf::Quaternion(0.589, 0.691, -0.259, 0.329));
        fixed_right_arm_rf.setOrigin(r_ee_pose.getOrigin());

        if (phase == PHASE_REACH_TO_PEEL){
            tf::Transform fixed_reach_to_peel_attr;
            fixed_reach_to_peel_attr.setOrigin(tf::Vector3(-0.148, -0.005, 0.247));
            fixed_reach_to_peel_attr.setRotation(tf::Quaternion(-0.549, 0.654, -0.284, 0.436));
            left_final_target.mult(fixed_right_arm_rf,fixed_reach_to_peel_attr);

        }else{

            tf::Transform fixed_peel_attr;
            fixed_peel_attr.setOrigin(tf::Vector3(-0.164, -0.025, 0.375));
            fixed_peel_attr.setRotation(tf::Quaternion(-0.503, 0.650, -0.307, 0.480));
            left_final_target.mult(fixed_right_arm_rf,fixed_peel_attr);
        }
    }
    else
        left_final_target.mult(task_frame, left_att);


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
    left_cdsRun->setObjectFrame(toMatrix4(model_task_frame));
    left_cdsRun->setAttractorFrame(toMatrix4(left_final_target));
    left_cdsRun->setCurrentEEPose(toMatrix4(l_curr_ee_pose));
    left_cdsRun->setDT(model_dt);
    left_cdsRun->setMotionParameters(0.5,1,1,reachingThreshold, masterType, slaveType);
    left_cdsRun->postInit();


    // Vairable for execution
    ros::Duration loop_rate(model_dt);
    tf::Pose r_mNextRobotEEPose = r_curr_ee_pose;
    tf::Pose l_mNextRobotEEPose = l_curr_ee_pose;

    double r_pos_err, r_ori_err, l_pos_err, l_ori_err;

    ROS_INFO("Execution started");
    while(ros::ok()) {

        // Setting Initial conditions
        if (initial_config == true){
            r_curr_ee_pose = r_ee_pose;
            l_curr_ee_pose = l_ee_pose;
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
        if (phase == PHASE_REACH_TO_PEEL){
                tf::Transform  l_ee_rot, ee_2_rob;
                l_ee_rot.setIdentity(); ee_2_rob.setIdentity();

                // Transform Attractor to Origin
                l_des_ee_pose.mult(left_final_target.inverse(),l_mNextRobotEEPose);

                // -> Apply Rotation (pi on Y in Origin RF)
                l_ee_rot.setBasis(tf::Matrix3x3(-1,0,0,0,-1,0,0,0,1)); //z (pi)
                l_des_ee_pose.mult(l_ee_rot,l_des_ee_pose);
                //l_ee_rot.setBasis(tf::Matrix3x3(1,0,0,0,cos,-sin,0,sin,cos)); //x
//                l_ee_rot.setBasis(tf::Matrix3x3(1,0,0,0,0,1,0,-1,0)); //x (-pi/2)
                l_ee_rot.setBasis(tf::Matrix3x3(1,0,0,0,-0.1736,0.98480,0,-0.98480,-0.1736)); //x (-100 dg)
                l_des_ee_pose.mult(l_ee_rot,l_des_ee_pose);

                // Old attractor
//                l_ee_rot.setBasis(tf::Matrix3x3(-1,0,0,0,1,0,0,0,-1)); //Y (pi)
//                l_des_ee_pose.mult(l_ee_rot,l_des_ee_pose);
//                                l_ee_rot.setBasis(tf::Matrix3x3(0.906,0.422,0,-0.422,0.906,0,0,0,1)); //Z -25%
//                l_des_ee_pose.mult(l_ee_rot,l_des_ee_pose);

                // -> Transform back to Robot
                l_des_ee_pose.mult(left_final_target,l_des_ee_pose);
                l_des_ee_pose.setRotation(l_curr_ee_pose.getRotation().slerp(left_final_target.getRotation(), 0.75) );

                // Don't Care about master
                if (r_pos_err > 0.02)
                        r_des_ee_pose = r_curr_ee_pose;
        }
        else
            l_des_ee_pose = l_mNextRobotEEPose;


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
            ROS_INFO_STREAM("POSITION ERROR REACHED!");
            if (phase == PHASE_PEEL){
                sendPose(r_curr_ee_pose, l_curr_ee_pose);
                break;
            }
            else if((r_ori_err < orientationThreshold) || isnan(r_ori_err)) {
                ROS_INFO_STREAM("RIGHT ORIENTATION ERROR REACHED!");
                if((l_ori_err < orientationThreshold) || isnan(l_ori_err)){
                    ROS_INFO_STREAM("LEFT ORIENTATION ERROR REACHED!");
                    sendPose(r_curr_ee_pose, l_curr_ee_pose);
                    break;
                }
            }
        }

        loop_rate.sleep();
    }
    delete right_cdsRun;
    delete left_cdsRun;

}
