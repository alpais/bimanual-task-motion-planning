/*
 * Copyright (C) 2016 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 *
 * decoupled_motion_planner.cpp
 *
 * Created on : Feb 6, 2015
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

//-- Custom ActionLib Stuff --//
#include "actionlib/server/simple_action_server.h"
#include <bimanual_action_planners/PLAN2CTRLAction.h>

//-- Message Types --//
#include <robohow_common_msgs/MotionPhase.h>
#include <robohow_common_msgs/MotionModel.h>
#include <robohow_common_msgs/GaussianMixtureModel.h>
#include <robohow_common_msgs/GaussianDistribution.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"

//-- CDS Stuff --//
#include "CDSExecution.h"

//-- Eigen Stuff --//
#include <Eigen/Core>
#include <Eigen/Geometry>


tf::Pose r_ee_pose, r_curr_ee_pose, r_des_ee_pose, l_ee_pose, l_curr_ee_pose, l_des_ee_pose;
Eigen::VectorXd r_ee_ft, r_curr_ee_ft, l_ee_ft, l_curr_ee_ft;
volatile bool isOkay;
string r_base_path, l_base_path, r_topic_ns, l_topic_ns;
string R_EE_STATE_POSE_TOPIC, R_EE_STATE_FT_TOPIC, R_EE_CMD_POSE_TOPIC, R_EE_CMD_FT_TOPIC;
string L_EE_STATE_POSE_TOPIC, L_EE_STATE_FT_TOPIC, L_EE_CMD_POSE_TOPIC, L_EE_CMD_FT_TOPIC;
tf::StampedTransform right_arm_base, left_arm_base;
bool initial_config = true, simulation;
int tf_count(0);
double reachingThreshold (0.01), orientationThreshold (0.02), model_dt (0.001); //Defaults: [m],[rad],[s]

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


class PLAN2CTRLAction
{
protected:

    // Pouring phases
    enum TaskPhase {
        PHASE1=1,
        PHASE2=2,
        PHASEREACH=3
    };

    ros::NodeHandle nh_;
    ros::Subscriber r_sub_, r_sub_ft_, l_sub_, l_sub_ft_;
    ros::Publisher  r_pub_, r_pub_ft_, l_pub_, l_pub_ft_;
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

    // Execute action from learned models
    bool learned_model_execution(TaskPhase phase, CDSController::DynamicsType masterType, CDSController::DynamicsType slaveType,
                                 double reachingThreshold, double orientationThreshold, double model_dt,
                                 tf::Transform trans_obj, tf::Transform trans_r_att, tf::Transform trans_l_att,
                                 std::string r_model_base_path, std::string l_model_base_path) {

        ROS_INFO_STREAM(" Right Model Path "            << r_model_base_path);
        ROS_INFO_STREAM(" Left Model Path "             << l_model_base_path);
        ROS_INFO_STREAM(" Execute Learned model: phase "<< phase);
        ROS_INFO_STREAM(" Reaching threshold "          << reachingThreshold);
        ROS_INFO_STREAM(" Orientation threshold "       << orientationThreshold);
        ROS_INFO_STREAM(" Model DT "                    << model_dt);

        // Convert attractors to world frame
        tf::Transform  right_final_target, left_final_target;
        right_final_target.mult(trans_obj, trans_r_att);
        left_final_target.mult(trans_obj, trans_l_att);


        // Setting Initial conditions
        if (initial_config == true){
            r_curr_ee_pose = r_ee_pose;
            l_curr_ee_pose = l_ee_pose;
        }

        // Initialize CDS for right arm
        CDSExecution *right_cdsRun = new CDSExecution;
        right_cdsRun->initSimple(r_model_base_path, phase);
        right_cdsRun->setObjectFrame(toMatrix4(trans_obj));
        right_cdsRun->setAttractorFrame(toMatrix4(trans_r_att));
        right_cdsRun->setCurrentEEPose(toMatrix4(r_curr_ee_pose));
        right_cdsRun->setDT(model_dt);
        right_cdsRun->setMotionParameters(0.5,1,1,reachingThreshold, masterType, slaveType);
        right_cdsRun->postInit();


        // Initialize CDS for left arm
        CDSExecution *left_cdsRun = new CDSExecution;
        left_cdsRun->initSimple(l_model_base_path, phase);
        left_cdsRun->setObjectFrame(toMatrix4(trans_obj));
        left_cdsRun->setAttractorFrame(toMatrix4(trans_l_att));
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
            br.sendTransform(tf::StampedTransform(trans_obj, ros::Time::now(), right_robot_frame, "/task_frame"));

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
            if(r_pos_err < reachingThreshold && (r_ori_err < orientationThreshold || isnan(r_ori_err)))
                if(l_pos_err < reachingThreshold && (l_ori_err < orientationThreshold || isnan(l_ori_err))) {
                    break;
                }
            loop_rate.sleep();
        }
        delete right_cdsRun;
        delete left_cdsRun;
        return ros::ok();
    }

public:

    PLAN2CTRLAction(std::string name) :
        as_(nh_, name, boost::bind(&PLAN2CTRLAction::executeCB, this, _1), false),
        action_name_(name)
    {
        r_ee_ft.resize(6);
        l_ee_ft.resize(6);
        as_.start();
    }


    ~PLAN2CTRLAction(void)
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

        ///////////////////////////////////////////////
        /////----- EXECUTE REQUESTED ACTION ------/////
        ///////////////////////////////////////////////


        // Use learned models to do shit
        if(goal->action_type=="LEARNED_MODEL"){
            TaskPhase phase;
            if(goal->action_name == "reach") {
                phase = PHASEREACH;
            } else if(goal->action_name == "phase1") {
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
            tf::Transform trans_obj, trans_r_att, trans_l_att;

            // Set transform for task frame
            trans_obj.setRotation(tf::Quaternion(goal->task_frame.rotation.x,goal->task_frame.rotation.y,
                                                 goal->task_frame.rotation.z,goal->task_frame.rotation.w));
            trans_obj.setOrigin(tf::Vector3(goal->task_frame.translation.x, goal->task_frame.translation.y,
                                            goal->task_frame.translation.z));

            // Set transform for right arm attractor
            trans_r_att.setRotation(tf::Quaternion(goal->right_attractor_frame.rotation.x,goal->right_attractor_frame.rotation.y,
                                                   goal->right_attractor_frame.rotation.z,goal->right_attractor_frame.rotation.w));
            trans_r_att.setOrigin(tf::Vector3(goal->right_attractor_frame.translation.x, goal->right_attractor_frame.translation.y,
                                              goal->right_attractor_frame.translation.z));

            // Set transform for left arm attractor
            trans_l_att.setRotation(tf::Quaternion(goal->left_attractor_frame.rotation.x,goal->left_attractor_frame.rotation.y,
                                                   goal->left_attractor_frame.rotation.z,goal->left_attractor_frame.rotation.w));
            trans_l_att.setOrigin(tf::Vector3(goal->left_attractor_frame.translation.x, goal->left_attractor_frame.translation.y,
                                              goal->left_attractor_frame.translation.z));

            // Execute action from learned action model
            success = learned_model_execution(phase, masterType, slaveType, reachingThreshold, orientationThreshold,
                                              model_dt, trans_obj, trans_r_att, trans_l_att, r_base_path, l_base_path);
        }

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
    PLAN2CTRLAction action_execution(ros::this_node::getName());
    action_execution.initialize();
    ros::spin();
    return 0;
}
