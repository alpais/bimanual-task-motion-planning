#include "bimanual_action_server.h"

bool BimanualActionServer::coupled_learned_model_execution(TaskPhase phase, CDSController::DynamicsType masterType,
                                                             CDSController::DynamicsType slaveType, double reachingThreshold,
                                                             double orientationThreshold, double model_dt, tf::Transform task_frame,
                                                             tf::Transform right_att, tf::Transform left_att, std::string r_model_base_path,
                                                             std::string l_model_base_path)
{

    ROS_INFO_STREAM(" Right Model Path "            << r_model_base_path);
    ROS_INFO_STREAM(" Left Model Path "             << l_model_base_path);
    ROS_INFO_STREAM(" Execute Learned model: phase "<< phase);
    ROS_INFO_STREAM(" Reaching threshold "          << reachingThreshold);
    ROS_INFO_STREAM(" Orientation threshold "       << orientationThreshold);
    ROS_INFO_STREAM(" Model DT "                    << model_dt);

    // Convert attractors to world frame
    tf::Transform  right_final_target, left_final_target;
    right_final_target.mult(task_frame, right_att);
    left_final_target.mult(task_frame, left_att);


    // Setting Initial conditions
    if (initial_config == true){
        r_curr_ee_pose = r_ee_pose;
        l_curr_ee_pose = l_ee_pose;
    }

    // Initialize CDS for right arm
    CDSExecution *right_cdsRun = new CDSExecution;
    right_cdsRun->initSimple(r_model_base_path, phase);
    right_cdsRun->setObjectFrame(toMatrix4(task_frame));
    right_cdsRun->setAttractorFrame(toMatrix4(right_att));
    right_cdsRun->setCurrentEEPose(toMatrix4(r_curr_ee_pose));
    right_cdsRun->setDT(model_dt);
    right_cdsRun->setMotionParameters(0.5,1,1,reachingThreshold, masterType, slaveType);
    right_cdsRun->postInit();


    // Initialize CDS for left arm
    CDSExecution *left_cdsRun = new CDSExecution;
    left_cdsRun->initSimple(l_model_base_path, phase);
    left_cdsRun->setObjectFrame(toMatrix4(task_frame));
    left_cdsRun->setAttractorFrame(toMatrix4(left_att));
    left_cdsRun->setCurrentEEPose(toMatrix4(l_curr_ee_pose));
    left_cdsRun->setDT(model_dt);
    left_cdsRun->setMotionParameters(0.5,1,1,reachingThreshold, masterType, slaveType);
    left_cdsRun->postInit();


    // Vairable for execution
    ros::Duration loop_rate(model_dt);
    static tf::TransformBroadcaster br;
    tf::Pose r_mNextRobotEEPose = r_curr_ee_pose;
    tf::Pose l_mNextRobotEEPose = l_curr_ee_pose;

    tf::Transform r_trans_ee, l_trans_ee;
    double r_pos_err, r_ori_err, l_pos_err, l_ori_err;


    ROS_INFO("Execution started");
    while(ros::ok()) {

        // Setting Initial conditions
        if (initial_config == true){
            r_curr_ee_pose = r_ee_pose;
            l_curr_ee_pose = l_ee_pose;
        }
        else{
            r_curr_ee_pose = r_des_ee_pose;
            l_curr_ee_pose = l_des_ee_pose;
        }

        // Publish attractors if running in simulation or with fixed values
        r_trans_ee.setRotation(tf::Quaternion(r_curr_ee_pose.getRotation()));
        r_trans_ee.setOrigin(tf::Vector3(r_curr_ee_pose.getOrigin()));

        l_trans_ee.setRotation(tf::Quaternion(l_curr_ee_pose.getRotation()));
        l_trans_ee.setOrigin(tf::Vector3(l_curr_ee_pose.getOrigin()));

        // To Visualize EE Frames
        if (simulation==true){
            int frame_viz = int(model_dt*1000);
            if (tf_count==0 || tf_count%frame_viz==0){
                stringstream r_ss, l_ss;
                r_ss <<  "/r_ee_tf_" << tf_count;
                l_ss <<  "/l_ee_tf_" << tf_count;
                br.sendTransform(tf::StampedTransform(r_trans_ee, ros::Time::now(), right_robot_frame, r_ss.str()));
                br.sendTransform(tf::StampedTransform(l_trans_ee, ros::Time::now(), right_robot_frame, l_ss.str()));
            }
            tf_count++;
        }
        else{
            br.sendTransform(tf::StampedTransform(r_trans_ee, ros::Time::now(), right_robot_frame, "/r_ee_tf"));
            br.sendTransform(tf::StampedTransform(l_trans_ee, ros::Time::now(), right_robot_frame, "/l_ee_tf"));
        }

        br.sendTransform(tf::StampedTransform(right_final_target, ros::Time::now(), right_robot_frame, "/right_attractor"));
        br.sendTransform(tf::StampedTransform(left_final_target, ros::Time::now(), right_robot_frame, "/left_attractor"));
        br.sendTransform(tf::StampedTransform(task_frame, ros::Time::now(), right_robot_frame, "/task_frame"));

        // Current progress variable (position/orientation error).
        r_pos_err = (right_final_target.getOrigin() - r_curr_ee_pose.getOrigin()).length();
        l_pos_err = (left_final_target.getOrigin()  - l_curr_ee_pose.getOrigin()).length();

        //Real Orientation Error qdiff = acos(dot(q1_norm,q2_norm))*180/pi
        r_ori_err = acos(abs(right_final_target.getRotation().dot(r_curr_ee_pose.getRotation())));
        l_ori_err = acos(abs(left_final_target.getRotation().dot(l_curr_ee_pose.getRotation())));
        ROS_INFO_STREAM_THROTTLE(0.5,"Position Threshold : "    << reachingThreshold    << " ... Current Right Error: " << r_pos_err << " Left Error: " << l_pos_err);
        ROS_INFO_STREAM_THROTTLE(0.5,"Orientation Threshold : " << orientationThreshold << " ... Current Right Error: " << r_ori_err << " Left Error: " << r_ori_err);

        double r_att_pos_err = (right_final_target.getOrigin() - r_des_ee_pose.getOrigin()).length();
        double r_att_ori_err = acos(abs(right_final_target.getRotation().dot(r_des_ee_pose.getRotation())));

        double l_att_pos_err = (left_final_target.getOrigin() - l_des_ee_pose.getOrigin()).length();
        double l_att_ori_err = acos(abs(left_final_target.getRotation().dot(l_des_ee_pose.getRotation())));

        ROS_INFO_STREAM_THROTTLE(0.5,"Des-Att Right Position Error: "    << r_att_pos_err << " Left Position Error: "    << l_att_pos_err);
        ROS_INFO_STREAM_THROTTLE(0.5,"Des-Att Right Orientation Error: " << r_att_ori_err << " Left Orientation Error: " << l_att_ori_err);

        // Compute Next Desired EE Pose for Right Arm
        right_cdsRun->setCurrentEEPose(toMatrix4(r_mNextRobotEEPose));
        toPose(right_cdsRun->getNextEEPose(), r_mNextRobotEEPose);
        r_des_ee_pose = r_mNextRobotEEPose;

        // Compute Next Desired EE Pose for Left Arm
        left_cdsRun->setCurrentEEPose(toMatrix4(l_mNextRobotEEPose));
        toPose(left_cdsRun->getNextEEPose(), l_mNextRobotEEPose);
        l_des_ee_pose = l_mNextRobotEEPose;

        // Make next pose the current pose for open-loop simulation
        if (simulation==true)
            initial_config=false;

        // CHECK If orientation error is VERY low or nan because of qdiff take target orientation
        if (r_att_ori_err < 0.005 || isnan(r_att_ori_err)) //[rad] and [m]//
            r_des_ee_pose.setRotation(tf::Quaternion(right_final_target.getRotation()).normalize());

        if (l_att_ori_err < 0.005 || isnan(l_att_ori_err)) //[rad] and [m]//
            l_des_ee_pose.setRotation(tf::Quaternion(left_final_target.getRotation()).normalize());

        //******************************//
        //  Send the computed ee poses  //
        //******************************//
        if (simulation==false)
            sendPose(r_des_ee_pose, l_des_ee_pose);

        as_.publishFeedback(feedback_);

        if(r_pos_err < reachingThreshold && l_pos_err < reachingThreshold){
            if (phase == PHASE_PEEL){
                sendPose(r_curr_ee_pose, l_curr_ee_pose);
                break;
            }
            else{
                if((r_ori_err < orientationThreshold || isnan(r_ori_err)) && (l_ori_err < orientationThreshold || isnan(l_ori_err))){
                    sendPose(r_curr_ee_pose, l_curr_ee_pose);
                    break;
                }
            }


        }

        loop_rate.sleep();
    }
    delete right_cdsRun;
    delete left_cdsRun;

//                if(phase ==  PHASE_REACH_TO_PEEL) {
//                    // Hack! If phase is "reach", find the table right after reaching
//                    if (bWaitForForces_right_arm)	{
//                        bool x_r_arm = find_object_by_contact(R_ARM_ID, 0.35, 0.05, 5);
//                        return x_r_arm;
//                    }
//                    if (bWaitForForces_left_arm)	{
//                        bool x_l_arm = find_object_by_contact(L_ARM_ID, 0.35, 0.05, 5);
//                        return x_l_arm;
//                    }
//                } else if (phase == PHASE_PEEL){
//                    // Hack! wait for zero force before getting ready to recieve further commands.
//                    // This is to avoid dragging the dough.
//                    sendAndWaitForNormalForce(0, L_ARM_ID);
//                    return ros::ok();
//                } else {
//                    return ros::ok();
//                }
}
