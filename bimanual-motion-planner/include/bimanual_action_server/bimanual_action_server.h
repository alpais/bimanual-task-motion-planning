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
#include "Eigen/Core"
#include "Eigen/Geometry"

//-- FT Sensors Stuff --//
#include "netft_rdt_driver/String_cmd.h"

#define MAX_PEELING_FORCE	15
#define FORCE_WAIT_TOL		9
#define R_ARM_ID            1
#define L_ARM_ID            2
#define DT                  0.002

class BimanualActionServer
{

protected:

    // Task phases
    enum TaskPhase {
        PHASE_INIT_REACH     = 0,
        PHASE_REACH_TO_PEEL  = 1,
        PHASE_PEEL           = 2,
        PHASE_ROTATE         = 3,
        PHASE_RETRACT        = 4,
    };


    ros::NodeHandle nh_;

    // Publishers + Subscribers
    ros::Subscriber r_sub_, r_sub_ft_, l_sub_, l_sub_ft_;
    ros::Publisher  r_pub_, r_pub_ft_, l_pub_, l_pub_ft_, ro_pub_, vo_pub_, vo_l_pub_, vo_r_pub_;
    string right_robot_frame, left_robot_frame;

    // Right/Left EE states/cmds/topics
    tf::Pose r_ee_pose, r_curr_ee_pose, r_des_ee_pose, l_ee_pose, l_curr_ee_pose, l_des_ee_pose;
    Eigen::VectorXd  r_curr_ee_ft, l_curr_ee_ft;
    string r_base_path, l_base_path, r_topic_ns, l_topic_ns;
    string R_EE_STATE_POSE_TOPIC, R_EE_STATE_FT_TOPIC, R_EE_CMD_POSE_TOPIC, R_EE_CMD_FT_TOPIC;
    string L_EE_STATE_POSE_TOPIC, L_EE_STATE_FT_TOPIC, L_EE_CMD_POSE_TOPIC, L_EE_CMD_FT_TOPIC;
    tf::StampedTransform right_arm_base, left_arm_base;

    // Service Clients
    ros::ServiceClient hand_ft_client;
    ros::ServiceClient tool_ft_client;

    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<bimanual_action_planners::PLAN2CTRLAction> as_;
    std::string action_name_;

    // create messages that are used to published feedback/result
    bimanual_action_planners::PLAN2CTRLFeedback feedback_;
    bimanual_action_planners::PLAN2CTRLResult result_;

    // Create messages for sending force commands
    geometry_msgs::WrenchStamped msg_ft;
    geometry_msgs::PoseStamped msg_pose;
    bool bWaitForForces_left_arm;
    bool bWaitForForces_right_arm;

    // Simulation/execution variables
    volatile bool isOkay;
    bool initial_config, simulation, just_visualize;
    int tf_count;
    double reachingThreshold, orientationThreshold, model_dt, dt; //Defaults: [m],[rad],[s]

    // Visualization variables for Bimanual DS Action
    visualization_msgs::Marker ro_marker;
    visualization_msgs::Marker vo_marker;
    visualization_msgs::Marker vo_arrow_left;
    visualization_msgs::Marker vo_arrow_right;


    //************************************//
    // FUNCTIONS USED BY ANY ACTION TYPE  //
    //***********************************//

    MathLib::Matrix4 toMatrix4(const tf::Pose& pose);

    void toPose(const MathLib::Matrix4& mat4, tf::Pose& pose);


    // Callback for the current right end effector pose
    void r_eeStateCallback(const geometry_msgs::PoseStampedConstPtr& msg);


    // Callback for the current right end effector force/torque
    void r_ftStateCallback(const geometry_msgs::WrenchStampedConstPtr& msg);

    // Callback for the current left end effector pose
    void l_eeStateCallback(const geometry_msgs::PoseStampedConstPtr& msg);


    // Callback for the current left end effector force/torque
    void l_ftStateCallback(const geometry_msgs::WrenchStampedConstPtr& msg);

    // Send desired EE_pose to robot/joint_ctrls.
    void sendPose(const tf::Pose& r_pose_, const tf::Pose& l_pose_);

    void sendPoseLeft(const tf::Pose& l_pose_);

    void sendPoseRight(const tf::Pose& r_pose_);

    // Send desired EE_ft to robot/joint_ctrls.
    void sendNormalForce(double fz, int arm_id);


    //************************************//
    // FUNCTIONS USED FOR FORCE CONTROL  //
    //***********************************//

    void biasFtSensors();

    // This will block until the desired force is achieved!
    void sendAndWaitForNormalForce(double fz, int arm_id);

    // Function to move ee in Z direction until contact is identified
    bool find_object_by_contact(int arm_id, double min_height, double vertical_speed, double thr_force);

    //**************************************//
    // FUNCTIONS USED BY VO DS ACTION TYPE  //
    //**************************************//

    // Compute Pose of Real Object
    void compute_object_pose(const tf::Transform& right, const tf::Transform& left, tf::Transform& object);

    // Publish real object shape for VO bimanual DS
    void publish_ro_rviz(const tf::Transform& ro_pose, const double& object_length);

    // Publish virtual object shape for VO bimanual DS
    void publish_vo_rviz(const tf::Transform& vo_pose,  const double& object_length, const tf::Transform& right, const tf::Transform& left);


    //*******************************************//
    // EXECUTION FUNCTIONS FOR EACH ACTION TYPE  //
    //*******************************************//


    // ACTION TYPE 1: Execute action from two independent learned models
    bool uncoupled_learned_model_execution(TaskPhase phase, CDSController::DynamicsType masterType, CDSController::DynamicsType slaveType,
                                           double reachingThreshold, double orientationThreshold, double model_dt, tf::Transform task_frame,
                                           tf::Transform right_att, tf::Transform left_att, std::string r_model_base_path,
                                           std::string l_model_base_path);

    // ACTION TYPE 2: Execute bimanual reach with virtual object dynamical system
    bool coordinated_bimanual_ds_execution(TaskPhase phase, tf::Transform task_frame, tf::Transform right_att, tf::Transform left_att, double dt);


    // ACTION TYPE 3: Execute bimanual reach with virtual object dynamical system
    bool coupled_learned_model_execution(TaskPhase phase, CDSController::DynamicsType masterType, CDSController::DynamicsType slaveType,
                                         double reachingThreshold, double orientationThreshold, double model_dt, tf::Transform task_frame,
                                         tf::Transform right_att, tf::Transform left_att, std::string r_model_base_path,
                                         std::string l_model_base_path);

    // ACTION TYPE 4: Go to Targets in Cartesian Space
    bool bimanual_goto_cart_execution(tf::Transform task_frame, tf::Transform right_att, tf::Transform left_att);

public:

    //**************************//
    // ACTION SERVER FUNCTIONS  //
    //**************************//

    BimanualActionServer(std::string name);

    ~BimanualActionServer(void);

    void initialize();

    void executeCB(const bimanual_action_planners::PLAN2CTRLGoalConstPtr &goal);


};
