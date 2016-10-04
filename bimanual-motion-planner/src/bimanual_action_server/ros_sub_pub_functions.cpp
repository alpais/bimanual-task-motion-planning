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
