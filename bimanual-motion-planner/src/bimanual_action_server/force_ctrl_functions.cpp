#include "bimanual_action_server.h"


void BimanualActionServer::biasFtSensors(){
    netft_rdt_driver::String_cmd srv;
    srv.request.cmd  = "bias";
    srv.response.res = "";
    if (hand_ft_client.call(srv))
    {
        ROS_INFO_STREAM("net_ft res: " << srv.response.res);
    }else{
        ROS_ERROR("Failed to call netft bias service for hand");
    }

    if (tool_ft_client.call(srv))
    {
        ROS_INFO_STREAM("net_ft res: " << srv.response.res);
    }else{
        ROS_ERROR("Failed to call netft bias service for tool");
    }
}



// This will block until the desired force is achieved!
void BimanualActionServer::sendAndWaitForNormalForce(double fz, int arm_id)
{
    if (arm_id==R_ARM_ID && bWaitForForces_right_arm){
        ROS_INFO_STREAM("Waiting for force on right arm "<<fz<<" N.");
        sendPose(r_ee_pose, l_ee_pose);

        ros::Rate wait(500);
        while(ros::ok()) {
            sendNormalForce(fz, arm_id);
            ROS_INFO_STREAM("Sending Normal force: " << fz << " Fz diff: " << fabs(r_curr_ee_ft[2]-fz));
            if(fabs(r_curr_ee_ft[2]-fz) < FORCE_WAIT_TOL) {
                break;
            }
            ros::spinOnce();
            wait.sleep();
        }
    }

    else if (arm_id==L_ARM_ID && bWaitForForces_left_arm){
        ROS_INFO_STREAM("Waiting for force on left arm "<<fz<<" N.");
        sendPose(r_ee_pose, l_ee_pose);

        ros::Rate wait(500);
        while(ros::ok()) {
            sendNormalForce(fz, arm_id);
            ROS_INFO_STREAM("Sending Normal force: " << fz << " Fz diff: " << fabs(l_curr_ee_ft[2]-fz));
            if(fabs(l_curr_ee_ft[2]-fz) < FORCE_WAIT_TOL) {
                break;
            }
            ros::spinOnce();
            wait.sleep();
        }
    }
}

bool BimanualActionServer::find_object_by_contact(int arm_id, double min_height, double vertical_speed, double thr_force) {
    double rate = 500;
    thr_force = fabs(thr_force);
    ros::Rate thread_rate(rate);
    Eigen::VectorXd ee_ft;
    ee_ft.resize(6);

    int left_arm = arm_id == L_ARM_ID;
    double offset_X = left_arm ? left_arm_base.getOrigin().getX() : 0;
    double offset_Y = left_arm ? left_arm_base.getOrigin().getY() : 0;
    double offset_Z = left_arm ? left_arm_base.getOrigin().getZ() : 0;

    // Figure out if it is the right arm or the left arm
    tf::Pose arm_pose;
    if (arm_id == R_ARM_ID){
        arm_pose.setOrigin(r_ee_pose.getOrigin());
        arm_pose.setRotation(r_ee_pose.getRotation());
    }
    else{
        arm_pose.setOrigin(l_ee_pose.getOrigin());
        arm_pose.setRotation(l_ee_pose.getRotation());
    }


    double startz = arm_pose.getOrigin().z();
    ROS_INFO_STREAM("Start Z:" << startz);

    msg_pose.pose.position.x = arm_pose.getOrigin().x() + offset_X;
    msg_pose.pose.position.y = arm_pose.getOrigin().y() + offset_Y;
    msg_pose.pose.position.z = arm_pose.getOrigin().z() + offset_Z;
    msg_pose.pose.orientation.x = arm_pose.getRotation().x();
    msg_pose.pose.orientation.y = arm_pose.getRotation().y();
    msg_pose.pose.orientation.z = arm_pose.getRotation().z();
    msg_pose.pose.orientation.w = arm_pose.getRotation().w();

    ROS_INFO_STREAM("Finding object up to max dist. "<< min_height <<" with vertical speed "<< vertical_speed <<" and threshold force "<<thr_force<<"N.");
    while(ros::ok()) {
        msg_pose.pose.position.z = msg_pose.pose.position.z - vertical_speed/rate;

        if (arm_id == R_ARM_ID){
            r_pub_.publish(msg_pose);
            ee_ft = r_curr_ee_ft;
        }
        else{
            l_pub_.publish(msg_pose);
            ee_ft = l_curr_ee_ft;
        }

        if (arm_id == R_ARM_ID){
            arm_pose.setOrigin(r_ee_pose.getOrigin());
            arm_pose.setRotation(r_ee_pose.getRotation());
        }
        else{
            arm_pose.setOrigin(l_ee_pose.getOrigin());
            arm_pose.setRotation(l_ee_pose.getRotation());
        }

        ROS_INFO_STREAM("Current force Z:" << ee_ft[2] << " and Z pos: " << fabs(arm_pose.getOrigin().z()-startz));

        // Go down until force reaches the threshold
        if(fabs(ee_ft[2]) > thr_force) {
            ROS_INFO("MAX Force applied");
            break;
        }
        if(fabs(arm_pose.getOrigin().z()-startz) > min_height) {
            ROS_INFO("Max distance reached");
            break;
        }
        thread_rate.sleep();
        feedback_.progress = ee_ft[2];
        as_.publishFeedback(feedback_);
    }
    if(!ros::ok()) {
        return false;
    }

    tf::Vector3 obj_to_touch(arm_pose.getOrigin());
    ROS_INFO_STREAM("Object for arm " << arm_id << " found at height " << obj_to_touch[2]);
    msg_pose.pose.position.z = obj_to_touch[2];

    if (arm_id == R_ARM_ID){
        r_pub_.publish(msg_pose);
    }
    else{
        l_pub_.publish(msg_pose);
    }

//    if(!simulation)
//        sendAndWaitForNormalForce(0, arm_id);

    return true;
}
