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

void BimanualActionServer::initializeForceModel(std::string base_path, TaskPhase phase, int arm_id, std::string role){

    string arm;
    if (arm_id == R_ARM_ID)
        arm = "Right";
    if (arm_id == L_ARM_ID)
        arm = "Left";


    if (arm_id == L_ARM_ID){
        if (bForceModelInitialized_l_arm){
            delete mForceModel_l_arm;
            bForceModelInitialized_l_arm = false;
        }
        else {
            char sForce[256];
            sprintf(sForce, "%s/Phase%d/%s/%s_posGMM.txt", base_path.c_str(), phase, arm.c_str(), role.c_str());

            std::vector<int> in_dim;  in_dim.resize(1);
            std::vector<int> out_dim; out_dim.resize(1);

            mForceModel_l_arm = new GMR(sForce);
            mForceModel_l_arm->initGMR(in_dim, out_dim);

            if (!mForceModel_l_arm->getIsInit()){
                ROS_INFO_STREAM("ERROR: Could not initialize force model for the " << arm << " arm");
                exit(1);
            }
            else
                bForceModelInitialized_l_arm = true;
        }
    }
    else if (arm_id == R_ARM_ID){
        if (bForceModelInitialized_r_arm){
            delete mForceModel_r_arm;
            bForceModelInitialized_r_arm = false;
        }
        else {
            char sForce[256];
            sprintf(sForce, "%s/Phase%d/%s/%s_posGMM.txt", base_path.c_str(), phase, arm.c_str(), role.c_str());

            std::vector<int> in_dim;  in_dim.resize(1);
            std::vector<int> out_dim; out_dim.resize(1);

            mForceModel_r_arm = new GMR(sForce);
            mForceModel_r_arm->initGMR(in_dim, out_dim);

            if (!mForceModel_r_arm->getIsInit()){
                ROS_INFO_STREAM("ERROR: Could not initialize force model for the " << arm << " arm");
                exit(1);
            }
            else
                bForceModelInitialized_r_arm = true;
        }

    } else {
        ROS_ERROR("Arm ID invalid. Could not initialize force from model");
        exit(1);
    }

}

//void initialize_cart_filter(double dt, double r_Wn, double l_Wn){
void BimanualActionServer::initialize_cart_filter(double dt, double r_Wn, double l_Wn){

    // Filter for right arm
    r_cdd_cart_filter = new CDDynamics(7, dt, r_Wn);
    MathLib::Vector r_vel_lim_cart(7);
    r_vel_lim_cart = DEG2RAD(60);
    r_cdd_cart_filter->SetVelocityLimits(r_vel_lim_cart);
    r_cdd_cart_filter->SetWn(r_Wn);

    // Filter for left arm
    l_cdd_cart_filter = new CDDynamics(7, dt, l_Wn);
    MathLib::Vector l_vel_lim_cart(7);
    l_vel_lim_cart = DEG2RAD(60);
    l_cdd_cart_filter->SetVelocityLimits(l_vel_lim_cart);
    l_cdd_cart_filter->SetWn(l_Wn);


}

void BimanualActionServer::sync_cart_filter(const tf::Pose& r_ee_pose, const tf::Pose& l_ee_pose){

    // synchronize Cartesian motion for the right arm
    MathLib::Vector r_init_state; r_init_state.Resize(7, false);
    r_init_state(0) = r_ee_pose.getOrigin().getX();
    r_init_state(1) = r_ee_pose.getOrigin().getY();
    r_init_state(2) = r_ee_pose.getOrigin().getZ();
    r_init_state(3) = r_ee_pose.getRotation().getX();
    r_init_state(4) = r_ee_pose.getRotation().getY();
    r_init_state(5) = r_ee_pose.getRotation().getZ();
    r_init_state(6) = r_ee_pose.getRotation().getW();
    r_cdd_cart_filter->SetState(r_init_state);

    // synchronize Cartesian motion for the left arm
    MathLib::Vector l_init_state; l_init_state.Resize(7, false);
    l_init_state(0) = l_ee_pose.getOrigin().getX();
    l_init_state(1) = l_ee_pose.getOrigin().getY();
    l_init_state(2) = l_ee_pose.getOrigin().getZ();
    l_init_state(3) = l_ee_pose.getRotation().getX();
    l_init_state(4) = l_ee_pose.getRotation().getY();
    l_init_state(5) = l_ee_pose.getRotation().getZ();
    l_init_state(6) = l_ee_pose.getRotation().getW();
    l_cdd_cart_filter->SetState(l_init_state);

}

void BimanualActionServer::filter_arm_motion(tf::Pose& r_des_ee_pose, tf::Pose& l_des_ee_pose){

    // Filter right arm motion
    MathLib::Vector r_next_target; r_next_target.Resize(7, false);
    r_next_target(0) = r_des_ee_pose.getOrigin().getX();
    r_next_target(1) = r_des_ee_pose.getOrigin().getY();
    r_next_target(2) = r_des_ee_pose.getOrigin().getZ();
    r_next_target(3) = r_des_ee_pose.getRotation().getX();
    r_next_target(4) = r_des_ee_pose.getRotation().getY();
    r_next_target(5) = r_des_ee_pose.getRotation().getZ();
    r_next_target(6) = r_des_ee_pose.getRotation().getW();

    r_cdd_cart_filter->SetTarget(r_next_target);
    r_cdd_cart_filter->Update();
    r_cdd_cart_filter->GetState(r_next_target);

    r_des_ee_pose.setOrigin(tf::Vector3(r_next_target(0), r_next_target(1), r_next_target(2)));
    r_des_ee_pose.setRotation(tf::Quaternion(r_next_target(3), r_next_target(4), r_next_target(5), r_next_target(6)));;

    // Filter left arm motion
    MathLib::Vector l_next_target; l_next_target.Resize(7, false);
    l_next_target(0) = l_des_ee_pose.getOrigin().getX();
    l_next_target(1) = l_des_ee_pose.getOrigin().getY();
    l_next_target(2) = l_des_ee_pose.getOrigin().getZ();
    l_next_target(3) = l_des_ee_pose.getRotation().getX();
    l_next_target(4) = l_des_ee_pose.getRotation().getY();
    l_next_target(5) = l_des_ee_pose.getRotation().getZ();
    l_next_target(6) = l_des_ee_pose.getRotation().getW();

    l_cdd_cart_filter->SetTarget(l_next_target);
    l_cdd_cart_filter->Update();
    l_cdd_cart_filter->GetState(l_next_target);

    l_des_ee_pose.setOrigin(tf::Vector3(l_next_target(0), l_next_target(1), l_next_target(2)));
    l_des_ee_pose.setRotation(tf::Quaternion(l_next_target(3), l_next_target(4), l_next_target(5), l_next_target(6)));;

}


// Send desired EE_pose to robot/joint_ctrls.
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



//****************************//
//   ---TYPE CONVERSIONS---   //
//****************************//

MathLib::Matrix4 BimanualActionServer::toMatrix4(const tf::Pose& pose) {
    MathLib::Matrix4 mat;
    mat.Identity();
    tf::Matrix3x3 mat33(pose.getRotation());

    mat.SetTranslation(MathLib::Vector3(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z()));
    mat.SetOrientation(MathLib::Matrix3(mat33[0][0], mat33[0][1], mat33[0][2],
            mat33[1][0], mat33[1][1], mat33[1][2],
            mat33[2][0], mat33[2][1], mat33[2][2]));
    return mat;
}

void BimanualActionServer::toPose(const MathLib::Matrix4& mat4, tf::Pose& pose) {
    MathLib::Matrix3 m1 = mat4.GetOrientation();
    MathLib::Vector3 v1 = m1.GetRotationAxis();
    tf::Vector3 ax(v1(0), v1(1), v1(2));
    tf::Quaternion q(ax, m1.GetRotationAngle());
    pose.setRotation(q.normalize());
    v1.Set(mat4.GetTranslation());
    pose.setOrigin(tf::Vector3(v1(0),v1(1),v1(2)));
}

