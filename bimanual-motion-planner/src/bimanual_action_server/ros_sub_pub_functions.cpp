#include "bimanual_action_server.h"

// Callback for the current right end effector pose
void BimanualActionServer::r_eeStateCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    const geometry_msgs::PoseStamped* data = msg.get();
    r_ee_pose.setOrigin(tf::Vector3(data->pose.position.x,data->pose.position.y,data->pose.position.z));
    r_ee_pose.setRotation(tf::Quaternion(data->pose.orientation.x,data->pose.orientation.y,data->pose.orientation.z,data->pose.orientation.w));
    isOkay = true;
}


// Callback for the current right end effector force/torque
void BimanualActionServer::r_ftStateCallback(const geometry_msgs::WrenchStampedConstPtr& msg) {
    const geometry_msgs::WrenchStamped* data = msg.get();
    r_curr_ee_ft[0] = data->wrench.force.x;
    r_curr_ee_ft[1] = data->wrench.force.y;
    r_curr_ee_ft[2] = data->wrench.force.z;

    r_curr_ee_ft[3] = data->wrench.torque.x;
    r_curr_ee_ft[4] = data->wrench.torque.y;
    r_curr_ee_ft[5] = data->wrench.torque.z;

}

void BimanualActionServer::r_jstiffStateCallback(const kuka_fri_bridge::JointStateImpedanceConstPtr &msg){

    r_curr_jstiff[0] = msg.get()->stiffness[0];
    r_curr_jstiff[1] = msg.get()->stiffness[1];
    r_curr_jstiff[2] = msg.get()->stiffness[2];
    r_curr_jstiff[3] = msg.get()->stiffness[3];
    r_curr_jstiff[4] = msg.get()->stiffness[4];
    r_curr_jstiff[5] = msg.get()->stiffness[5];
    r_curr_jstiff[6] = msg.get()->stiffness[6];

}

void BimanualActionServer::r_cartStiffStateCallback(const geometry_msgs::TwistConstPtr &msg){

    r_curr_cart_stiff[0] = msg.get()->linear.x;
    r_curr_cart_stiff[1] = msg.get()->linear.y;
    r_curr_cart_stiff[2] = msg.get()->linear.z;

}


// Callback for the current left end effector pose
void BimanualActionServer::l_eeStateCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    const geometry_msgs::PoseStamped* data = msg.get();
    l_ee_pose.setOrigin(tf::Vector3(data->pose.position.x - left_arm_base.getOrigin().getX(),data->pose.position.y - left_arm_base.getOrigin().getY(),data->pose.position.z -  left_arm_base.getOrigin().getZ()));
    l_ee_pose.setRotation(tf::Quaternion(data->pose.orientation.x,data->pose.orientation.y,data->pose.orientation.z,data->pose.orientation.w));
    isOkay = true;
}


// Callback for the current left end effector force/torque
void BimanualActionServer::l_ftStateCallback(const geometry_msgs::WrenchStampedConstPtr& msg) {
    const geometry_msgs::WrenchStamped* data = msg.get();
    l_curr_ee_ft[0] = data->wrench.force.x;
    l_curr_ee_ft[1] = data->wrench.force.y;
    l_curr_ee_ft[2] = data->wrench.force.z;

    l_curr_ee_ft[3] = data->wrench.torque.x;
    l_curr_ee_ft[4] = data->wrench.torque.y;
    l_curr_ee_ft[5] = data->wrench.torque.z;

}


void BimanualActionServer::l_jstiffStateCallback(const kuka_fri_bridge::JointStateImpedanceConstPtr &msg){

    l_curr_jstiff[0] = msg.get()->stiffness[0];
    l_curr_jstiff[1] = msg.get()->stiffness[1];
    l_curr_jstiff[2] = msg.get()->stiffness[2];
    l_curr_jstiff[3] = msg.get()->stiffness[3];
    l_curr_jstiff[4] = msg.get()->stiffness[4];
    l_curr_jstiff[5] = msg.get()->stiffness[5];
    l_curr_jstiff[6] = msg.get()->stiffness[6];
}

void BimanualActionServer::l_cartStiffStateCallback(const geometry_msgs::TwistConstPtr &msg){

    l_curr_cart_stiff[0] = msg.get()->linear.x;
    l_curr_cart_stiff[1] = msg.get()->linear.y;
    l_curr_cart_stiff[2] = msg.get()->linear.z;

}


// ------ Human arm and task frame tracked by vision ------ //
void BimanualActionServer::h_wristStateCallback(const geometry_msgs::PoseStampedConstPtr& msg){

    const geometry_msgs::PoseStamped* data = msg.get();
    vision_wrist_frame.setOrigin(tf::Vector3(data->pose.position.x,data->pose.position.y,data->pose.position.z));
    vision_wrist_frame.setRotation(tf::Quaternion(data->pose.orientation.x,data->pose.orientation.y,data->pose.orientation.z,data->pose.orientation.w));

}

void BimanualActionServer::h_taskFrameStateCallback(const geometry_msgs::PoseStampedConstPtr& msg){
    const geometry_msgs::PoseStamped* data = msg.get();
    vision_bowl_frame.setOrigin(tf::Vector3(data->pose.position.x,data->pose.position.y,data->pose.position.z));
    vision_bowl_frame.setRotation(tf::Quaternion(data->pose.orientation.x,data->pose.orientation.y,data->pose.orientation.z,data->pose.orientation.w));
    bEnableVision = true;
}

void BimanualActionServer::h_currentActionStateCallback(const std_msgs::BoolConstPtr& msg){

    h_current_action_state = msg.get()->data;

}


void BimanualActionServer::h_currentActionErrorCallback(const std_msgs::Float64ConstPtr &msg){

    h_current_action_err = msg.get()->data;

}


void BimanualActionServer::h_pub_crt_dist_err(tf::Vector3 &h_dist_err){

    geometry_msgs::Vector3 h_msg;

    h_msg.x = h_dist_err.getX();
    h_msg.y = h_dist_err.getY();
    h_msg.z = h_dist_err.getZ();

    h_dist_pub_.publish(h_msg);


}

// ------ Sending commands ---------------- //
void BimanualActionServer::sendPose(const tf::Pose& r_pose_, const tf::Pose& l_pose_) {

    sendPoseRight(r_pose_);
    sendPoseLeft(l_pose_);

}


void BimanualActionServer::sendPoseLeft(const tf::Pose& l_pose_) {
    geometry_msgs::PoseStamped  l_msg;

    // Populate left ee message
    l_msg.pose.position.x     = l_pose_.getOrigin().x() + left_arm_base.getOrigin().getX();
    l_msg.pose.position.y     = l_pose_.getOrigin().y() + left_arm_base.getOrigin().getY();
    l_msg.pose.position.z     = l_pose_.getOrigin().z() + left_arm_base.getOrigin().getZ();

    l_msg.pose.orientation.x  = l_pose_.getRotation().x();
    l_msg.pose.orientation.y  = l_pose_.getRotation().y();
    l_msg.pose.orientation.z  = l_pose_.getRotation().z();
    l_msg.pose.orientation.w  = l_pose_.getRotation().w();

    // Send left ee commands
    l_pub_.publish(l_msg);
}

void BimanualActionServer::sendPoseRight(const tf::Pose& r_pose_) {
    geometry_msgs::PoseStamped r_msg;

    // Populate right ee message
    r_msg.pose.position.x     = r_pose_.getOrigin().x();
    r_msg.pose.position.y     = r_pose_.getOrigin().y();
    r_msg.pose.position.z     = r_pose_.getOrigin().z();

    r_msg.pose.orientation.x  = r_pose_.getRotation().x();
    r_msg.pose.orientation.y  = r_pose_.getRotation().y();
    r_msg.pose.orientation.z  = r_pose_.getRotation().z();
    r_msg.pose.orientation.w  = r_pose_.getRotation().w();

    // Send right ee commands
    r_pub_.publish(r_msg);
}


// Send desired EE_FT to robot/joint_ctrls.
void BimanualActionServer::sendNormalForce(double fz, int arm_id) {
    msg_ft.wrench.force.x = 0;
    msg_ft.wrench.force.y = 0;
    msg_ft.wrench.force.z = fz;

    msg_ft.wrench.torque.x = 0;
    msg_ft.wrench.torque.y = 0;
    msg_ft.wrench.torque.z = 0;

    if (arm_id == R_ARM_ID){
        r_pub_ft_.publish(msg_ft);
    }
    else{
        l_pub_ft_.publish(msg_ft);
    }

}

// Send desired joint stiffness command
void BimanualActionServer::sendJStiffCmd(double des_stiff, int arm_id){

    // Here we bypass the state-transformer and we update the stiffness on our own

//    ROS_INFO_STREAM("Publishing Joint Stiff on topic " << R_CMD_STIFF_TOPIC);

    // use same stiffness for all axes
    jstiff_msg.stiffness.resize(7);
    jstiff_msg.stiffness[0] = des_stiff;      jstiff_msg.stiffness[1] = des_stiff;      jstiff_msg.stiffness[2] = des_stiff;
    jstiff_msg.stiffness[3] = des_stiff;      jstiff_msg.stiffness[4] = des_stiff;      jstiff_msg.stiffness[5] = des_stiff;
    jstiff_msg.stiffness[6] = des_stiff;

    if (arm_id == R_ARM_ID){
        r_pub_jstiff_.publish(jstiff_msg);
    }
    else{
        l_pub_jstiff_.publish(jstiff_msg);
    }

}

void BimanualActionServer::sendCartStiffCmd(Eigen::Vector3d des_stiff, int arm_id){

    // Here we bypass the state-transformer and we update the stiffness on our own

    msg_cart_stiff.linear.x = des_stiff[0];
    msg_cart_stiff.linear.y = des_stiff[0];
    msg_cart_stiff.linear.z = des_stiff[0];

    if (arm_id == R_ARM_ID){
        r_pub_cart_stiff_.publish(msg_cart_stiff);
    }
    else{
        l_pub_cart_stiff_.publish(msg_cart_stiff);
    }

}


void BimanualActionServer::publish_task_frames(tf::Pose &r_curr_ee_pose, tf::Pose &l_curr_ee_pose,
                                               tf::Transform &right_final_target, tf::Transform &left_final_target,
                                               tf::Transform &task_frame)
{

    static tf::TransformBroadcaster br;
    tf::Transform r_trans_ee, l_trans_ee;
    r_trans_ee.setRotation(tf::Quaternion(r_curr_ee_pose.getRotation()));
    r_trans_ee.setOrigin(tf::Vector3(r_curr_ee_pose.getOrigin()));

    l_trans_ee.setRotation(tf::Quaternion(l_curr_ee_pose.getRotation()));
    l_trans_ee.setOrigin(tf::Vector3(l_curr_ee_pose.getOrigin()));

    // To Visualize EE Frames
    if (just_visualize==true){
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


}


void BimanualActionServer::gloveAndTekscanUpdateCallback(const glove_tekscan_ros_wrapper::LasaDataStreamWrapperConstPtr& msg){

    // Using directly the average values for each patch
    thumb_pressure  (0)  = msg.get()->thumb1_f_avg;    thumb_pressure   (1)  = msg.get()->thumb2_f_avg;
    index_pressure  (0)  = msg.get()->index1_f_avg;    index_pressure   (1)  = msg.get()->index2_f_avg;    index_pressure   (2)  = msg.get()->index3_f_avg;
    middle_pressure (0)  = msg.get()->middle1_f_avg;   middle_pressure  (1)  = msg.get()->middle2_f_avg;   middle_pressure  (2)  = msg.get()->middle3_f_avg;
    ring_pressure   (0)  = msg.get()->ring1_f_avg;     ring_pressure    (1)  = msg.get()->ring2_f_avg;     ring_pressure    (2)  = msg.get()->ring3_f_avg;
    pinky_pressure  (0)  = msg.get()->pinky1_f_avg;    pinky_pressure   (1)  = msg.get()->pinky2_f_avg;    pinky_pressure   (2)  = msg.get()->pinky3_f_avg;
    palm_pressure   (0)  = msg.get()->palm1_f_avg;     palm_pressure    (1)  = msg.get()->palm2_f_avg;

    avg_fingertip_pressure = (thumb_pressure(0) + index_pressure(0) + middle_pressure(0) + ring_pressure(0) + pinky_pressure(0))/5;

    // Computing averages per finger
    avg_thumb_pressure  = thumb_pressure.Sum()/thumb_pressure.Size();
    avg_index_pressure  = index_pressure.Sum()/index_pressure.Size();
    avg_middle_pressure = middle_pressure.Sum()/middle_pressure.Size();
    avg_ring_pressure   = ring_pressure.Sum()/ring_pressure.Size();
    avg_pinky_pressure  = pinky_pressure.Sum()/pinky_pressure.Size();
    avg_palm_pressure   = palm_pressure.Sum()/palm_pressure.Size();

    // Joint angles -- Check the mapping in Matlab;
    thumb_ja(0) = msg.get()->finger_pos[2]; // thumb flexion
    thumb_ja(1) = msg.get()->finger_pos[3]; // thumb abduction
    thumb_ja(2) = msg.get()->finger_pos[4]; // thumb roll

    index_ja(0) = msg.get()->finger_pos[5]; // index flexion
    index_ja(1) = msg.get()->finger_pos[6]; // index abduction
    index_ja(2) = msg.get()->finger_pos[7]; // index proximal
    index_ja(3) = msg.get()->finger_pos[8]; // index distal

    middle_ja(0) = msg.get()->finger_pos[9];  // middle flexion
    middle_ja(1) = msg.get()->finger_pos[10]; // middle abduction
    middle_ja(2) = msg.get()->finger_pos[11]; // middle proximal
    middle_ja(3) = msg.get()->finger_pos[12]; // middle distal

    ring_ja(0) = msg.get()->finger_pos[13];   // ring flexion
    ring_ja(1) = msg.get()->finger_pos[14];   // ring abduction
    ring_ja(2) = msg.get()->finger_pos[15];   // ring proximal
    ring_ja(3) = msg.get()->finger_pos[16];   // ring distal

    pinky_ja(0) = msg.get()->finger_pos[17]; // pinky flexion
    pinky_ja(1) = msg.get()->finger_pos[18]; // pinky abduction
    pinky_ja(2) = msg.get()->finger_pos[19]; // pinky proximal
    pinky_ja(3) = msg.get()->finger_pos[20]; // pinky distal

    palm_ja(0) = msg.get()->finger_pos[21];


    finger_joints_all( 0) = msg.get()->finger_pos[0];   finger_joints_all( 1) = msg.get()->finger_pos[1];   finger_joints_all( 2) = msg.get()->finger_pos[2];
    finger_joints_all( 3) = msg.get()->finger_pos[3];   finger_joints_all( 4) = msg.get()->finger_pos[4];   finger_joints_all( 5) = msg.get()->finger_pos[5];
    finger_joints_all( 6) = msg.get()->finger_pos[6];   finger_joints_all( 7) = msg.get()->finger_pos[7];   finger_joints_all( 8) = msg.get()->finger_pos[8];
    finger_joints_all( 9) = msg.get()->finger_pos[9];   finger_joints_all(10) = msg.get()->finger_pos[10];  finger_joints_all(11) = msg.get()->finger_pos[11];
    finger_joints_all(12) = msg.get()->finger_pos[12];  finger_joints_all(13) = msg.get()->finger_pos[13];  finger_joints_all(14) = msg.get()->finger_pos[14];
    finger_joints_all(15) = msg.get()->finger_pos[15];  finger_joints_all(16) = msg.get()->finger_pos[16];  finger_joints_all(17) = msg.get()->finger_pos[17];
    finger_joints_all(18) = msg.get()->finger_pos[18];  finger_joints_all(19) = msg.get()->finger_pos[19];  finger_joints_all(20) = msg.get()->finger_pos[20];
    finger_joints_all(21) = msg.get()->finger_pos[21];

    bGloveTekscanInitialized = true;

}

void BimanualActionServer::initialize_ros_publishers_and_subscribers(){

    std::stringstream r_ss_state_pose, r_ss_state_ft, r_ss_state_stiff;
    std::stringstream r_ss_cmd_pose, r_ss_cmd_ft, r_ss_cmd_stiff;

    std::stringstream l_ss_state_pose, l_ss_state_ft, l_ss_state_stiff;
    std::stringstream l_ss_cmd_pose, l_ss_cmd_ft, l_ss_cmd_stiff;


#ifdef USE_JOINT_CONTROLLERS        // The state transformers package will transform cartesian commands to joint commands

    // Right Arm
    r_ss_state_pose << "/" << r_topic_ns << "/joint_to_cart/est_ee_pose";
    r_ss_state_ft   << "/hand/ft_sensor/netft_data";
    r_ss_state_stiff<< "/KUKA_RightArm/joint_imp_states";           // Read directly from the robot_mirror

    r_ss_cmd_pose   << "/" << r_topic_ns << "/cart_to_joint/des_ee_pose";
    r_ss_cmd_ft     << "/" << r_topic_ns << "/cart_to_joint/des_ee_ft";
    r_ss_cmd_stiff  << "/KUKA_RightArm/joint_imp_cmd";
//    r_ss_cmd_stiff  << "/" << r_topic_ns << "/cart_to_joint/des_ee_stiff";

    // Left Arm
    l_ss_state_pose << "/" << l_topic_ns << "/joint_to_cart/est_ee_pose";
    l_ss_state_ft   << "/tool/ft_sensor/netft_data";
    l_ss_state_stiff<< "/KUKA_LeftArm/joint_imp_states";            // Read directly from the robot_mirror

    l_ss_cmd_pose   << "/" << l_topic_ns << "/cart_to_joint/des_ee_pose";
    l_ss_cmd_ft     << "/" << l_topic_ns << "/cart_to_joint/des_ee_ft";
    l_ss_cmd_stiff  << "/KUKA_LeftArm/joint_imp_cmd";
//    l_ss_cmd_stiff  << "/" << l_topic_ns << "/cart_to_joint/des_ee_stiff";

/*   -------------------------------------------------------------------------------
 *   Note that the stiffness command needs to be sent directly to the robot_mirror.
 *   In the current implementation of the state_transformers, sending the command
 *   to /left_arm/cart_to_joint/des_ee_stiff will not update the stiffness.
 *   -------------------------------------------------------------------------------
 */

#endif

#ifdef USE_FRI_CART_CONTROLLERS         // Bypass the state transformers completely

    // Right Arm
    r_ss_state_pose         << "/KUKA_RightArm/Pose";
    r_ss_state_ft           << "/hand/ft_sensor/netft_data";
    r_ss_state_stiff        << "/KUKA_RightArm/Stiff"

    r_ss_cmd_pose           << "/KUKA_RightArm/des_ee_pose";
    r_ss_cmd_ft             << "/KUKA_RightArm/des_ee_ft";
    r_ss_cmd_stiff          << "/KUKA_RightArm/des_ee_stiff";

    // Left Arm
    l_ss_state_pose         << "/KUKA_LeftArm/Pose";
    l_ss_state_ft           << "/tool/ft_sensor/netft_data";    // or alternatively take the estimate from the robot >> /KUKA_LeftArm/FT
    l_ss_state_stiff        << "/KUKA_LeftArm/Stiff";

    l_ss_cmd_pose           << "/KUKA_LeftArm/des_ee_pose";
    l_ss_cmd_ft             << "/KUKA_LeftArm/des_ee_ft";
    l_ss_cmd_stiff          << "/KUKA_LeftArm/des_ee_stiff";

#endif

    // ----- >> Vision
    std::stringstream vis_ss_bowl_pose, vis_ss_wrist_pose;
    vis_ss_bowl_pose << "Bowl_Frame/pose";
    vis_ss_wrist_pose << "Human_Wrist/pose";

    VISION_BOWL_POSE_TOPIC = vis_ss_bowl_pose.str();
    VISION_WRIST_POSE_TOPIC = vis_ss_wrist_pose.str();

    // ----- >> Human
    std::stringstream h_ss_state_dist;
    h_ss_state_dist     << "h_estim/dist_to_att";
    H_STATE_DIST_TOPIC    = h_ss_state_dist.str();

    std::stringstream   h_ss_glove_tekscan;
    h_ss_glove_tekscan << "/LasaDataStream";
    H_STATE_GLOVE_TEKSCAN   = h_ss_glove_tekscan.str();

    // ----- >> Right Arm
    R_EE_STATE_POSE_TOPIC = r_ss_state_pose.str();
    R_EE_STATE_FT_TOPIC	  = r_ss_state_ft.str();
    R_STATE_STIFF_TOPIC   = r_ss_state_stiff.str();

    R_EE_CMD_POSE_TOPIC	  = r_ss_cmd_pose.str();
    R_EE_CMD_FT_TOPIC	  = r_ss_cmd_ft.str();
    R_CMD_STIFF_TOPIC     = r_ss_cmd_stiff.str();

    // ----- >> Left Arm
    L_EE_STATE_POSE_TOPIC = l_ss_state_pose.str();
    L_EE_STATE_FT_TOPIC	  = l_ss_state_ft.str();
    L_STATE_STIFF_TOPIC   = l_ss_state_stiff.str();

    L_EE_CMD_POSE_TOPIC	  = l_ss_cmd_pose.str();
    L_EE_CMD_FT_TOPIC	  = l_ss_cmd_ft.str();
    L_CMD_STIFF_TOPIC     = l_ss_cmd_stiff.str();

    // ROS TOPICS for right arm controllers
    r_sub_    = nh_.subscribe<geometry_msgs::PoseStamped>(R_EE_STATE_POSE_TOPIC, 1, &BimanualActionServer::r_eeStateCallback, this);
    r_pub_    = nh_.advertise<geometry_msgs::PoseStamped>(R_EE_CMD_POSE_TOPIC, 1);

    r_sub_ft_ = nh_.subscribe<geometry_msgs::WrenchStamped>(R_EE_STATE_FT_TOPIC, 1, &BimanualActionServer::r_ftStateCallback, this);
    r_pub_ft_ = nh_.advertise<geometry_msgs::WrenchStamped>(R_EE_CMD_FT_TOPIC, 1);

    r_sub_jstiff_ = nh_.subscribe(R_STATE_STIFF_TOPIC, 1, &BimanualActionServer::r_jstiffStateCallback, this);
    r_pub_jstiff_ = nh_.advertise<kuka_fri_bridge::JointStateImpedance>(R_CMD_STIFF_TOPIC, 1);

    // ROS TOPICS for left arm controllers
    l_sub_    = nh_.subscribe<geometry_msgs::PoseStamped>(L_EE_STATE_POSE_TOPIC, 1, &BimanualActionServer::l_eeStateCallback, this);
    l_sub_ft_ = nh_.subscribe<geometry_msgs::WrenchStamped>(L_EE_STATE_FT_TOPIC, 1, &BimanualActionServer::l_ftStateCallback, this);

    l_pub_    = nh_.advertise<geometry_msgs::PoseStamped>(L_EE_CMD_POSE_TOPIC, 1);
    l_pub_ft_ = nh_.advertise<geometry_msgs::WrenchStamped>(L_EE_CMD_FT_TOPIC, 1);

    l_sub_jstiff_ = nh_.subscribe(L_STATE_STIFF_TOPIC, 1, &BimanualActionServer::l_jstiffStateCallback, this);
    l_pub_jstiff_ = nh_.advertise<kuka_fri_bridge::JointStateImpedance>(L_CMD_STIFF_TOPIC, 1);

    // ROS TOPICS for vision
    vision_wrist_pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>(VISION_BOWL_POSE_TOPIC, 1, &BimanualActionServer::h_taskFrameStateCallback, this);
    vision_bowl_pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>(VISION_WRIST_POSE_TOPIC, 1, &BimanualActionServer::h_wristStateCallback, this);

    // ROS TOPICS for human state
    thumb_pressure.Resize(3); index_pressure.Resize(3); middle_pressure.Resize(3); ring_pressure.Resize(3); pinky_pressure.Resize(3); palm_pressure.Resize(2);
    thumb_ja.Resize(3); index_ja.Resize(4); middle_ja.Resize(4); ring_ja.Resize(4); pinky_ja.Resize(4); palm_ja.Resize(1);
    finger_joints_all.Resize(nFingerJoints); finger_joints_mask.Resize(nFingerJoints); finger_joints_avg.Resize(nFingerJoints); finger_joints_deltas.Resize(nFingerJoints);

    h_action_state_sub_ = nh_.subscribe<std_msgs::Bool>("state_estimator/action_state", 1, &BimanualActionServer::h_currentActionStateCallback, this);
    h_glove_and_tekscan_sub_ = nh_.subscribe<glove_tekscan_ros_wrapper::LasaDataStreamWrapper>(H_STATE_GLOVE_TEKSCAN, 1, &BimanualActionServer::gloveAndTekscanUpdateCallback, this);
    h_dist_pub_ = nh_.advertise<geometry_msgs::Vector3>(H_STATE_DIST_TOPIC, 1);

#ifdef USE_FRI_CART_CONTROLLERS
    r_sub_cart_stiff_ = nh_.subscribe<geometry_msgs::TwistStamped>(R_STATE_STIFF_TOPIC, 1, &BimanualActionServer::r_cartStiffStateCallback, this);
    r_pub_cart_stiff_ = nh_.advertise<geometry_msgs::TwistStamped>(R_CMD_STIFF_TOPIC,1);

    l_sub_cart_stiff_ = nh_.subscribe<geometry_msgs::TwistStamped>(L_STATE_STIFF_TOPIC, 1, &BimanualActionServer::l_cartStiffStateCallback, this);
    l_pub_cart_stiff_ = nh_.advertise<geometry_msgs::TwistStamped>(L_CMD_STIFF_TOPIC,1);
#endif

    r_curr_ee_ft.resize(6); r_curr_jstiff.resize(nDOF);  r_curr_cart_stiff.resize(3);
    l_curr_ee_ft.resize(6); l_curr_jstiff.resize(nDOF);  l_curr_cart_stiff.resize(3);

    ROS_INFO_STREAM("Right - Passive - FT Sensor: " << R_EE_STATE_FT_TOPIC);
    ROS_INFO_STREAM("Left - Active Tool -  FT Sensor: " << L_EE_STATE_FT_TOPIC);

    // ROS PUBLISHERS FOR VIRTUAL AND REAL OBJECT SHAPES
    ro_pub_   = nh_.advertise<visualization_msgs::Marker>("real_object", 1);
    vo_pub_   = nh_.advertise<visualization_msgs::Marker>("virtual_object", 1);
    vo_l_pub_ = nh_.advertise<visualization_msgs::Marker>("left_vo", 1);
    vo_r_pub_ = nh_.advertise<visualization_msgs::Marker>("right_vo", 1);


    // Real object marker settings
    ro_marker.header.frame_id = right_robot_frame;
    ro_marker.action = visualization_msgs::Marker::ADD;
    ro_marker.ns = "basic_shapes";
    ro_marker.id = 0;
    ro_marker.type = visualization_msgs::Marker::CUBE;

    ro_marker.color.r = 0.0f;
    ro_marker.color.g = 0.0f;
    ro_marker.color.b = 1.0f;
    ro_marker.color.a = 0.6f;

    // Virtual object marker settings
    vo_marker.header.frame_id = right_robot_frame;
    vo_marker.action = visualization_msgs::Marker::ADD;
    vo_marker.ns = "basic_shapes";
    vo_marker.id = 1;
    vo_marker.type = visualization_msgs::Marker::CUBE;
    vo_marker.color.r = 0.0f;
    vo_marker.color.g = 1.0f;
    vo_marker.color.b = 0.0f;
    vo_marker.color.a = 0.6f;

    vo_arrow_left.header.frame_id = right_robot_frame;
    vo_arrow_left.action = visualization_msgs::Marker::ADD;
    vo_arrow_left.ns = "basic_shapes";
    vo_arrow_left.id = 2;
    vo_arrow_left.type = visualization_msgs::Marker::ARROW;
    vo_arrow_left.color.r = 1.0f;
    vo_arrow_left.color.g = 0.0f;
    vo_arrow_left.color.b = 0.0f;
    vo_arrow_left.color.a = 0.6f;
    vo_arrow_left.scale.x = 0.005;
    vo_arrow_left.scale.y = 0.01;
    vo_arrow_left.scale.z = 0.01;

    vo_arrow_right.header.frame_id = right_robot_frame;
    vo_arrow_right.action = visualization_msgs::Marker::ADD;
    vo_arrow_right.ns = "basic_shapes";
    vo_arrow_right.id = 3;
    vo_arrow_right.type = visualization_msgs::Marker::ARROW;
    vo_arrow_right.color.r = 1.0f;
    vo_arrow_right.color.g = 0.0f;
    vo_arrow_right.color.b = 0.0f;
    vo_arrow_right.color.a = 0.6f;
    vo_arrow_right.scale.x = 0.005;
    vo_arrow_right.scale.y = 0.01;
    vo_arrow_right.scale.z = 0.01;



}
