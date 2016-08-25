/*
 * Copyright (C) 2016 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 *
 * bimanual_action_server.cpp
 *
 * Created on : April 12, 2016
 * Author     : nbfigueroa
 * Email      : nadia.figueroafernandez@epfl.ch
 * Website    : lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <ros/ros.h>

//-- TF Stuff --//
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

//-- Custom ActionLib Stuff --//
#include "actionlib/server/simple_action_server.h"
#include <bimanual_action_planners/PLAN2CTRLAction.h>

//-- Message Types --//
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"

//-- CDS Stuff --//
#include "CDSExecution.h"

//-- Virtual Object Stuff --//
#include "bimanual_ds_execution.h"
#include <visualization_msgs/Marker.h>

//-- Eigen Stuff --//
#include <Eigen/Core>
#include <Eigen/Geometry>


// Right/Left EE states/cmds/topics
tf::Pose r_ee_pose, r_curr_ee_pose, r_des_ee_pose, l_ee_pose, l_curr_ee_pose, l_des_ee_pose;
Eigen::VectorXd r_ee_ft, r_curr_ee_ft, l_ee_ft, l_curr_ee_ft;
string r_base_path, l_base_path, r_topic_ns, l_topic_ns;
string R_EE_STATE_POSE_TOPIC, R_EE_STATE_FT_TOPIC, R_EE_CMD_POSE_TOPIC, R_EE_CMD_FT_TOPIC;
string L_EE_STATE_POSE_TOPIC, L_EE_STATE_FT_TOPIC, L_EE_CMD_POSE_TOPIC, L_EE_CMD_FT_TOPIC;
tf::StampedTransform right_arm_base, left_arm_base;

// Simulation/execution variables
volatile bool isOkay;
bool initial_config = true, simulation;
int tf_count(0);
double reachingThreshold (0.01), orientationThreshold (0.1), model_dt (0.002), dt (0.002); //Defaults: [m],[rad],[s]

// Visualization variables for Bimanual DS Action
uint32_t   shape = visualization_msgs::Marker::CUBE;
uint32_t   arrow = visualization_msgs::Marker::ARROW;
visualization_msgs::Marker ro_marker;
visualization_msgs::Marker vo_marker;
visualization_msgs::Marker vo_arrow_left;
visualization_msgs::Marker vo_arrow_right;

// Variables for Bimanual DS
#define WN_CART_PLANNER   (70.0)


//************************************//
// FUNCTIONS USED BY ANY ACTION TYPE  //
//***********************************//

// Callback for the current right end effector pose
void r_eeStateCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    const geometry_msgs::PoseStamped* data = msg.get();
    r_ee_pose.setOrigin(tf::Vector3(data->pose.position.x,data->pose.position.y,data->pose.position.z));
    r_ee_pose.setRotation(tf::Quaternion(data->pose.orientation.x,data->pose.orientation.y,data->pose.orientation.z,data->pose.orientation.w));
    isOkay = true;
}


// Callback for the current right end effector force/torque
void r_ftStateCallback(const geometry_msgs::WrenchStampedConstPtr& msg) {
    const geometry_msgs::WrenchStamped* data = msg.get();
    r_curr_ee_ft[0] = data->wrench.force.x;
    r_curr_ee_ft[1] = data->wrench.force.y;
    r_curr_ee_ft[2] = data->wrench.force.z;

    r_curr_ee_ft[3] = data->wrench.torque.x;
    r_curr_ee_ft[4] = data->wrench.torque.y;
    r_curr_ee_ft[5] = data->wrench.torque.z;
}



// Callback for the current left end effector pose
void l_eeStateCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    const geometry_msgs::PoseStamped* data = msg.get();
    l_ee_pose.setOrigin(tf::Vector3(data->pose.position.x - left_arm_base.getOrigin().getX(),data->pose.position.y - left_arm_base.getOrigin().getY(),data->pose.position.z -  left_arm_base.getOrigin().getZ()));
    l_ee_pose.setRotation(tf::Quaternion(data->pose.orientation.x,data->pose.orientation.y,data->pose.orientation.z,data->pose.orientation.w));
    isOkay = true;
}


// Callback for the current left end effector force/torque
void l_ftStateCallback(const geometry_msgs::WrenchStampedConstPtr& msg) {
    const geometry_msgs::WrenchStamped* data = msg.get();
    l_curr_ee_ft[0] = data->wrench.force.x;
    l_curr_ee_ft[1] = data->wrench.force.y;
    l_curr_ee_ft[2] = data->wrench.force.z;

    l_curr_ee_ft[3] = data->wrench.torque.x;
    l_curr_ee_ft[4] = data->wrench.torque.y;
    l_curr_ee_ft[5] = data->wrench.torque.z;
}


MathLib::Matrix4 toMatrix4(const tf::Pose& pose) {
    MathLib::Matrix4 mat;
    mat.Identity();
    tf::Matrix3x3 mat33(pose.getRotation());

    mat.SetTranslation(MathLib::Vector3(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z()));
    mat.SetOrientation(MathLib::Matrix3(mat33[0][0], mat33[0][1], mat33[0][2],
            mat33[1][0], mat33[1][1], mat33[1][2],
            mat33[2][0], mat33[2][1], mat33[2][2]));
    return mat;
}

void toPose(const MathLib::Matrix4& mat4, tf::Pose& pose) {
    MathLib::Matrix3 m1 = mat4.GetOrientation();
    MathLib::Vector3 v1 = m1.GetRotationAxis();
    tf::Vector3 ax(v1(0), v1(1), v1(2));
    tf::Quaternion q(ax, m1.GetRotationAngle());
    pose.setRotation(q.normalize());
    v1.Set(mat4.GetTranslation());
    pose.setOrigin(tf::Vector3(v1(0),v1(1),v1(2)));
}


void compute_object_pose(const tf::Transform& right, const tf::Transform& left, tf::Transform& object){

    // Compute object position (midpoint between left and right reaching points)
    tf::Vector3 o_pos = left.getOrigin() + (right.getOrigin() - left.getOrigin())/2;
    tf::Vector3 tmp(right.getOrigin());
//    tmp.setX(tmp.getX() + 0.05);
    tmp.setX(tmp.getX() + 0.05);

    // Compute object orientation (xdir: direction from right to left reaching point, ydir: y plane in world rf)
    Eigen::Vector3d xdir (right.getOrigin().getX() - left.getOrigin().getX(), right.getOrigin().getY() - left.getOrigin().getY(), right.getOrigin().getZ() - left.getOrigin().getZ());

    // Construct a plane "parallel" to the x dir of the robot base (table) and get the normal for the zdir
    Eigen::Hyperplane<double,3> p = Eigen::Hyperplane<double,3>::Through(Eigen::Vector3d(right.getOrigin().getX(),right.getOrigin().getY(),right.getOrigin().getZ()),
              Eigen::Vector3d(left.getOrigin().getX(),left.getOrigin().getY(),left.getOrigin().getZ()),
              Eigen::Vector3d(tmp.getX(),tmp.getY(),tmp.getZ()));
    p.normalize();
    Eigen::Vector3d zdir = p.normal();

    Eigen::Vector3d Rx = xdir/xdir.norm();
    Eigen::Vector3d Rz = zdir/zdir.norm();
    Eigen::Vector3d Ry = Rz.cross(Rx)/(Rz.cross(Rx)).norm();

    Eigen::Matrix3d R;
    R << Rx, Ry, Rz;
    tf::Matrix3x3 tf3d;
    tf::matrixEigenToTF(R,tf3d);
    tf::Quaternion o_rot;
    tf3d.getRotation(o_rot);

    // Set position and orientation of object
    object.setRotation(o_rot);
    object.setOrigin(o_pos);

}


class BimanualActionServer
{
protected:

    // Task phases
    enum TaskPhase {
        PHASE1=1,
        PHASE2=2
    };

    ros::NodeHandle nh_;
    ros::Subscriber r_sub_, r_sub_ft_, l_sub_, l_sub_ft_;
    ros::Publisher  r_pub_, r_pub_ft_, l_pub_, l_pub_ft_, ro_pub_, vo_pub_, vo_l_pub_, vo_r_pub_;
    string right_robot_frame, left_robot_frame;


    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<bimanual_action_planners::PLAN2CTRLAction> as_;
    std::string action_name_;

    // create messages that are used to published feedback/result
    bimanual_action_planners::PLAN2CTRLFeedback feedback_;
    bimanual_action_planners::PLAN2CTRLResult result_;

    // Send desired EE_pose to robot/joint_ctrls.
    void sendPose(const tf::Pose& r_pose_, const tf::Pose& l_pose_) {
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


    // Publish real object shape for VO bimanual DS
    void publish_ro_rviz(const tf::Transform& ro_pose, const double& object_length){

        // Marker 6DOF pose
        ro_marker.pose.position.x    = ro_pose.getOrigin().getX();
        ro_marker.pose.position.y    = ro_pose.getOrigin().getY();
        ro_marker.pose.position.z    = ro_pose.getOrigin().getZ();
        ro_marker.pose.orientation.x = ro_pose.getRotation().getX();
        ro_marker.pose.orientation.y = ro_pose.getRotation().getY();
        ro_marker.pose.orientation.z = ro_pose.getRotation().getZ();
        ro_marker.pose.orientation.w = ro_pose.getRotation().getW();

        // Marker shape
        ro_marker.scale.x = object_length;
        ro_marker.scale.y = 0.01;
        ro_marker.scale.z = 0.01;

        // Publish shape
        ro_pub_.publish(ro_marker);

    }


    // Publish virtual object shape for VO bimanual DS
    void publish_vo_rviz(const tf::Transform& vo_pose,  const double& object_length, const tf::Transform& right, const tf::Transform& left){

        // Marker 6DOF pose
        vo_marker.pose.position.x    = vo_pose.getOrigin().getX();
        vo_marker.pose.position.y    = vo_pose.getOrigin().getY();
        vo_marker.pose.position.z    = vo_pose.getOrigin().getZ();
        vo_marker.pose.orientation.x = vo_pose.getRotation().getX();
        vo_marker.pose.orientation.y = vo_pose.getRotation().getY();
        vo_marker.pose.orientation.z = vo_pose.getRotation().getZ();
        vo_marker.pose.orientation.w = vo_pose.getRotation().getW();

        // Marker shape
        vo_marker.scale.x = object_length;
        vo_marker.scale.y = 0.01;
        vo_marker.scale.z = 0.01;

        // Publish shape
        vo_pub_.publish(vo_marker);

        // Compute end-points of VO
        tf::Transform left_pose (vo_pose.inverse()), right_pose (vo_pose.inverse()) ;
        tf::Vector3 pos_l (left_pose.getOrigin()), pos_r (right_pose.getOrigin());
        double object_length_2 = object_length / 2;
        pos_l.setX(left_pose.getOrigin().getX() - object_length_2);
        pos_r.setX(right_pose.getOrigin().getX() + object_length_2);

        // Left VO endpoint
        left_pose.setOrigin(pos_l);
        left_pose.mult(vo_pose, left_pose);
        left_pose.mult(left_pose.inverse(),vo_pose);

        // Right VO endpoint
        right_pose.setOrigin(pos_r);
        right_pose.mult(vo_pose, right_pose);
        right_pose.mult(right_pose.inverse(),vo_pose);

        // Publish left arrow
        vo_arrow_left.points.resize(2);
        vo_arrow_left.points[0].x = left_pose.getOrigin().getX();
        vo_arrow_left.points[0].y = left_pose.getOrigin().getY();
        vo_arrow_left.points[0].z = left_pose.getOrigin().getZ();
        vo_arrow_left.points[1].x = left.getOrigin().getX();
        vo_arrow_left.points[1].y = left.getOrigin().getY();
        vo_arrow_left.points[1].z = left.getOrigin().getZ();
        vo_l_pub_.publish(vo_arrow_left);


        // Publish right arrow
        vo_arrow_right.points.resize(2);
        vo_arrow_right.points[0].x = right_pose.getOrigin().getX();
        vo_arrow_right.points[0].y = right_pose.getOrigin().getY();
        vo_arrow_right.points[0].z = right_pose.getOrigin().getZ();
        vo_arrow_right.points[1].x = right.getOrigin().getX();
        vo_arrow_right.points[1].y = right.getOrigin().getY();
        vo_arrow_right.points[1].z = right.getOrigin().getZ();
        vo_r_pub_.publish(vo_arrow_right);

    }


    // ACTION TYPE 1: Execute action from two independent learned models
    bool decoupled_learned_model_execution(TaskPhase phase, CDSController::DynamicsType masterType, CDSController::DynamicsType slaveType,
                                 double reachingThreshold, double orientationThreshold, double model_dt,
                                 tf::Transform task_frame, tf::Transform right_att, tf::Transform left_att,
                                 std::string r_model_base_path, std::string l_model_base_path) {

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
                if (phase == PHASE2)
                    break;
                else
                    if((r_ori_err < orientationThreshold || isnan(r_ori_err)) && (l_ori_err < orientationThreshold || isnan(l_ori_err)))
                        break;
            }

//            if(r_pos_err < reachingThreshold && (r_ori_err < orientationThreshold || isnan(r_ori_err)))
//                if(l_pos_err < reachingThreshold && (l_ori_err < orientationThreshold || isnan(l_ori_err))) {
//                    break;
//                }
            loop_rate.sleep();
        }
        delete right_cdsRun;
        delete left_cdsRun;
        return ros::ok();
    }

    // ACTION TYPE 2: Execute bimanual reach with virtual object dynamical system
    bool coordinated_bimanual_ds_execution(tf::Transform task_frame, tf::Transform right_att, tf::Transform left_att, double dt){

        // Convert attractors to world frame
        tf::Transform  right_final_target, left_final_target, virtual_object, real_object;
        right_final_target.mult(task_frame, right_att);
        left_final_target.mult(task_frame, left_att);

        // Compute Real object frame from desired left/right attractors
        compute_object_pose(left_final_target, right_final_target, real_object);
        double object_length (left_final_target.getOrigin().distance(right_final_target.getOrigin()));

        // State Variables for Virtial Object Dynamical System
        Eigen::Vector3d real_object_velocity;
        real_object_velocity.setZero();

        // Setting Initial conditions
        r_curr_ee_pose = r_ee_pose;
        l_curr_ee_pose = l_ee_pose;

        // Initialize Virtual Object Dynamical System
        bimanual_ds_execution *vo_dsRun = new bimanual_ds_execution;
        vo_dsRun->init(dt,0.5,0.5,800.0,400.0,400.0);
        vo_dsRun->setCurrentObjectState(real_object, real_object_velocity);
        vo_dsRun->setInterceptPositions(real_object, left_final_target, right_final_target);
        vo_dsRun->setCurrentEEStates(l_curr_ee_pose,r_curr_ee_pose);
        vo_dsRun->initializeVirtualObject();

        // Vairable for execution
        ros::Duration loop_rate(dt);
        static tf::TransformBroadcaster br;
        tf::Transform r_trans_ee, l_trans_ee;
        double object_err;

        while(ros::ok()) {

            // View attractors
            br.sendTransform(tf::StampedTransform(right_final_target, ros::Time::now(), right_robot_frame, "/right_attractor"));
            br.sendTransform(tf::StampedTransform(left_final_target, ros::Time::now(), right_robot_frame, "/left_attractor"));
            br.sendTransform(tf::StampedTransform(real_object, ros::Time::now(), right_robot_frame, "/real_object"));

            // View real object
            publish_ro_rviz(real_object, object_length);

            // Set Current Object State (From Object Pose Tracking/Predicting Module)
            vo_dsRun->setCurrentObjectState(real_object, real_object_velocity);

            // Set Predicted Intercept Positions (From Intercept Point Estimation Module)
            vo_dsRun->setInterceptPositions(real_object, left_final_target, right_final_target);

            // Update Current robot end-effector poses (From Fwd Kinematics)
            r_curr_ee_pose = r_ee_pose;
            l_curr_ee_pose = l_ee_pose;

            // Set Current ee States
            vo_dsRun->setCurrentEEStates(l_curr_ee_pose,r_curr_ee_pose);

            // Update VO DS
            vo_dsRun->update();

            // Get New Virtual Object Pose
            vo_dsRun->getVirtualObjectPose(virtual_object);

            // Get Desired ee States
            vo_dsRun->getNextEEStates(l_des_ee_pose,r_des_ee_pose);


            //******************************//
            //  Send the computed ee poses  //
            //******************************//
            sendPose(r_des_ee_pose, l_des_ee_pose);

            // Visualize desired end-effector poses
            r_trans_ee.setRotation(tf::Quaternion(r_des_ee_pose.getRotation()));
            r_trans_ee.setOrigin(tf::Vector3(r_des_ee_pose.getOrigin()));

            l_trans_ee.setRotation(tf::Quaternion(l_des_ee_pose.getRotation()));
            l_trans_ee.setOrigin(tf::Vector3(l_des_ee_pose.getOrigin()));

            br.sendTransform(tf::StampedTransform(r_trans_ee, ros::Time::now(), right_robot_frame, "/r_des_ee"));
            br.sendTransform(tf::StampedTransform(l_trans_ee, ros::Time::now(), right_robot_frame, "/l_des_ee"));

            //View virtual object
            publish_vo_rviz(virtual_object, object_length, r_curr_ee_pose, l_curr_ee_pose);
            br.sendTransform(tf::StampedTransform(virtual_object, ros::Time::now(), right_robot_frame, "/virtual_object"));

            // Open Loop Hack
            // r_curr_ee_pose = r_des_ee_pose;
            // l_curr_ee_pose = l_des_ee_pose;


            // Current progress variable (position)
            object_err = (virtual_object.getOrigin() - real_object.getOrigin()).length();  
//            reachingThreshold = 0.012;
            ROS_INFO_STREAM_THROTTLE(0.5,"Position Threshold : "    << reachingThreshold    << " ... Current VO Error: " << object_err); 

            as_.publishFeedback(feedback_);

            // // Only Check for Position Error
            if(object_err < reachingThreshold) {
                    break;
                }

            loop_rate.sleep();
        }
        delete vo_dsRun;
        return ros::ok();
    }


    // ACTION TYPE 3: Execute collision avoidance between master/slave arm with virtual object dynamical system
    bool collision_avoidance_vo_execution(tf::Transform task_frame, tf::Transform right_att, tf::Transform left_att){

        // Convert attractors to world frame
        tf::Transform  right_final_target, left_final_target, virtual_object;
        right_final_target.mult(task_frame, right_att);
        left_final_target.mult(task_frame, left_att);

    }


public:

    BimanualActionServer(std::string name) :
        as_(nh_, name, boost::bind(&BimanualActionServer::executeCB, this, _1), false),
        action_name_(name)
    {
        r_ee_ft.resize(6);
        l_ee_ft.resize(6);
        as_.start();
    }


    ~BimanualActionServer(void)
    {
    }

    // Select which mode to use from launch file. Fixed numbers for LASA/BOXY robot or the Vision tracking.
    void initialize() {
        std::string ad;
        ros::NodeHandle _nh("~");

        // Read Parameters from Launch File
        _nh.getParam("right_robot_frame", right_robot_frame);
        _nh.getParam("left_robot_frame", left_robot_frame);        
        _nh.getParam("right_model_base_path", r_base_path);
        _nh.getParam("left_model_base_path", l_base_path);
        _nh.getParam("simulation", simulation);
        _nh.getParam("model_dt", model_dt);
        _nh.getParam("reachingThreshold", reachingThreshold);
        _nh.getParam("orientationThreshold", orientationThreshold);
        _nh.getParam("r_topic_ns", r_topic_ns);
        _nh.getParam("l_topic_ns", l_topic_ns);


        std::stringstream r_ss_state_pose, r_ss_state_ft, r_ss_cmd_pose, r_ss_cmd_ft;
        r_ss_state_pose << "/" << r_topic_ns << "/joint_to_cart/est_ee_pose";
        r_ss_state_ft   << "/" << r_topic_ns << "/joint_to_cart/est_ee_ft";
        r_ss_cmd_pose   << "/" << r_topic_ns << "/cart_to_joint/des_ee_pose";
        r_ss_cmd_ft     << "/" << r_topic_ns << "/cart_to_joint/des_ee_ft";

        R_EE_STATE_POSE_TOPIC = r_ss_state_pose.str();
        R_EE_STATE_FT_TOPIC	  = r_ss_state_ft.str();
        R_EE_CMD_POSE_TOPIC	  = r_ss_cmd_pose.str();
        R_EE_CMD_FT_TOPIC	  = r_ss_cmd_ft.str();

        // ROS TOPICS for right arm controllers
        r_sub_    = nh_.subscribe<geometry_msgs::PoseStamped>(R_EE_STATE_POSE_TOPIC, 1, r_eeStateCallback);
        r_sub_ft_ = nh_.subscribe<geometry_msgs::WrenchStamped>(R_EE_STATE_FT_TOPIC, 1, r_ftStateCallback);
        r_pub_    = nh_.advertise<geometry_msgs::PoseStamped>(R_EE_CMD_POSE_TOPIC, 1);
        r_pub_ft_ = nh_.advertise<geometry_msgs::WrenchStamped>(R_EE_CMD_FT_TOPIC, 1);


        std::stringstream l_ss_state_pose, l_ss_state_ft, l_ss_cmd_pose, l_ss_cmd_ft;
        l_ss_state_pose << "/" << l_topic_ns << "/joint_to_cart/est_ee_pose";
        l_ss_state_ft   << "/" << l_topic_ns << "/joint_to_cart/est_ee_ft";
        l_ss_cmd_pose   << "/" << l_topic_ns << "/cart_to_joint/des_ee_pose";
        l_ss_cmd_ft     << "/" << l_topic_ns << "/cart_to_joint/des_ee_ft";

        L_EE_STATE_POSE_TOPIC = l_ss_state_pose.str();
        L_EE_STATE_FT_TOPIC	  = l_ss_state_ft.str();
        L_EE_CMD_POSE_TOPIC	  = l_ss_cmd_pose.str();
        L_EE_CMD_FT_TOPIC	  = l_ss_cmd_ft.str();


        // ROS TOPICS for left arm controllers
        l_sub_    = nh_.subscribe<geometry_msgs::PoseStamped>(L_EE_STATE_POSE_TOPIC, 1, l_eeStateCallback);
        l_sub_ft_ = nh_.subscribe<geometry_msgs::WrenchStamped>(L_EE_STATE_FT_TOPIC, 1, l_ftStateCallback);
        l_pub_    = nh_.advertise<geometry_msgs::PoseStamped>(L_EE_CMD_POSE_TOPIC, 1);
        l_pub_ft_ = nh_.advertise<geometry_msgs::WrenchStamped>(L_EE_CMD_FT_TOPIC, 1);


        // Getting left and right robot base frame
        tf::TransformListener listener;
        try {
            listener.waitForTransform(right_robot_frame, "/world_frame", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform(right_robot_frame, "/world_frame", ros::Time(0), right_arm_base);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }
        try {
            listener.waitForTransform(left_robot_frame, right_robot_frame, ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform(left_robot_frame, right_robot_frame, ros::Time(0), left_arm_base);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }


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

    void executeCB(const bimanual_action_planners::PLAN2CTRLGoalConstPtr &goal)
    {

        std::string desired_action = goal->action_type;
        ROS_INFO_STREAM( "Desired Action is " << desired_action);

        isOkay = false;
        ros::Rate r(10);
        ROS_INFO("Waiting for EE pose/ft topic...");
        while(ros::ok() && (!isOkay)) {
            r.sleep();
        }
        if(!ros::ok()) {
            result_.success = 0;
            ROS_INFO("%s: Failed", action_name_.c_str());
            as_.setAborted(result_);
            return;
        }

        // initialize action progress as null
        feedback_.progress = 0;
        bool success = false;


        // Setup transforms for task-space control
        tf::Transform task_frame, right_att, left_att;
        // Set transform for task frame
        task_frame.setRotation(tf::Quaternion(goal->task_frame.rotation.x,goal->task_frame.rotation.y,
                                             goal->task_frame.rotation.z,goal->task_frame.rotation.w));
        task_frame.setOrigin(tf::Vector3(goal->task_frame.translation.x, goal->task_frame.translation.y,
                                        goal->task_frame.translation.z));

        // Set transform for right arm attractor
        right_att.setRotation(tf::Quaternion(goal->right_attractor_frame.rotation.x,goal->right_attractor_frame.rotation.y,
                                               goal->right_attractor_frame.rotation.z,goal->right_attractor_frame.rotation.w));
        right_att.setOrigin(tf::Vector3(goal->right_attractor_frame.translation.x, goal->right_attractor_frame.translation.y,
                                          goal->right_attractor_frame.translation.z));

        // Set transform for left arm attractor
        left_att.setRotation(tf::Quaternion(goal->left_attractor_frame.rotation.x,goal->left_attractor_frame.rotation.y,
                                               goal->left_attractor_frame.rotation.z,goal->left_attractor_frame.rotation.w));
        left_att.setOrigin(tf::Vector3(goal->left_attractor_frame.translation.x, goal->left_attractor_frame.translation.y,
                                          goal->left_attractor_frame.translation.z));


        ////////////////////////////////////////////////////
        /////----- EXECUTE REQUESTED ACTION TYPE ------/////
        ////////////////////////////////////////////////////

        //---> ACTION TYPE 1: Use two independent decoupled learned models to execute the action
        if(goal->action_type=="DECOUPLED_LEARNED_MODEL"){
            TaskPhase phase;
            if(goal->action_name == "phase1") {
                phase = PHASE1;
            } else if(goal->action_name == "phase2") {
                phase = PHASE2;
            } else {
                ROS_ERROR_STREAM("Unidentified action name "<<goal->action_name.c_str());
                result_.success = 0;
                as_.setAborted(result_);
                return;
            }

            CDSController::DynamicsType masterType = CDSController::MODEL_DYNAMICS;
            CDSController::DynamicsType slaveType = CDSController::UTHETA;



            // Execute action from learned action model
            success = decoupled_learned_model_execution(phase, masterType, slaveType, reachingThreshold, orientationThreshold,
                                              model_dt, task_frame, right_att, left_att, r_base_path, l_base_path);
        }

        //---> ACTION TYPE 2: Use the virtual object dynamical system to execute a bimanual reach
        if(goal->action_type=="BIMANUAL_REACH")
            success = coordinated_bimanual_ds_execution(task_frame, right_att, left_att, dt);

        //---> ACTION TYPE 3: Use coupled learned models to execute the action
        if(goal->action_type=="COLL_AVOID_VO")
            success = collision_avoidance_vo_execution(task_frame, right_att, left_att);


        result_.success = success;
        if(success)
        {
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            as_.setSucceeded(result_);
        } else {
            ROS_INFO("%s: Failed", action_name_.c_str());
            as_.setAborted(result_);
        }

    }
};


int main(int argc, char** argv) {

    ros::init(argc, argv, "bimanual_plan2ctrl");
    ROS_INFO("Initializing Server");
    BimanualActionServer action_execution(ros::this_node::getName());
    action_execution.initialize();
    ros::spin();
    return 0;
}
