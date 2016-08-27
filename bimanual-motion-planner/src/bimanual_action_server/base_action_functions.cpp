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

// Send desired EE_pose to robot/joint_ctrls.
void BimanualActionServer::sendPose(const tf::Pose& r_pose_, const tf::Pose& l_pose_) {
    geometry_msgs::PoseStamped r_msg, l_msg;

    // Populate right ee message
    r_msg.pose.position.x     = r_pose_.getOrigin().x();
    r_msg.pose.position.y     = r_pose_.getOrigin().y();
    r_msg.pose.position.z     = r_pose_.getOrigin().z();

    r_msg.pose.orientation.x  = r_pose_.getRotation().x();
    r_msg.pose.orientation.y  = r_pose_.getRotation().y();
    r_msg.pose.orientation.z  = r_pose_.getRotation().z();
    r_msg.pose.orientation.w  = r_pose_.getRotation().w();

    // Populate left ee message
    l_msg.pose.position.x     = l_pose_.getOrigin().x() + left_arm_base.getOrigin().getX();
    l_msg.pose.position.y     = l_pose_.getOrigin().y() + left_arm_base.getOrigin().getY();
    l_msg.pose.position.z     = l_pose_.getOrigin().z() + left_arm_base.getOrigin().getZ();

    l_msg.pose.orientation.x  = l_pose_.getRotation().x();
    l_msg.pose.orientation.y  = l_pose_.getRotation().y();
    l_msg.pose.orientation.z  = l_pose_.getRotation().z();
    l_msg.pose.orientation.w  = l_pose_.getRotation().w();

    // Send right and left ee commands
    r_pub_.publish(r_msg);
    l_pub_.publish(l_msg);
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

