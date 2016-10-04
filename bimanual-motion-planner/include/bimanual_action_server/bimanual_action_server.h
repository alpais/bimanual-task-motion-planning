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

//-- CDDynamics filtering --//
#include "CDDynamics.h"

//-- Stiffness -- //
#include "kuka_fri_bridge/JointStateImpedance.h"


#define FORCE_WAIT_TOL		7
#define R_ARM_ID            1
#define L_ARM_ID            2
#define DT                  0.002
#define R_ARM_ROLE          "master"
#define L_ARM_ROLE          "slave"

#define PEELING_TASK_ID     1
#define SCOOPING_TASK_ID    2

#define SEARCH_DIR_X        0
#define SEARCH_DIR_Y        1
#define SEARCH_DIR_Z        2

#define FT_CTRL_AXIS_X                      0
#define FT_CTRL_AXIS_Y                      1
#define FT_CTRL_AXIS_Z                      2
#define FT_CTRL_AXIS_RX                     3
#define FT_CTRL_AXIS_RY                     4
#define FT_CTRL_AXIS_RZ                     5


// Define active task >> In the future read this from file
// #define CRT_TASK_SCOOPING
// #define CRT_TASK_PEELING

class BimanualActionServer
{

protected:

    bool bDisplayDebugInfo;

    bool bBypassOri; // Compute the orientation using CDDynamics
    bool bFilterOri; // Smooth the orientation given by CDS or VO
    bool bIgnoreOri; // Completly discard the orientation, only control for position

    enum TaskPhase
    {
        // PEELING Task phases
        PHASE_INIT_REACH            =  0,
        PHASE_REACH_TO_PEEL         =  1,
        PHASE_PEEL                  =  2,
        PHASE_ROTATE                =  3,
        PHASE_RETRACT               =  4,
        // SCOOPING Task phases
        PHASE_SCOOP_INIT_REACH      =  5,
        PHASE_SCOOP_REACH_TO_SCOOP  =  6,
        PHASE_SCOOP_SCOOP           =  7,
        PHASE_SCOOP_DEPART          =  8,
        PHASE_SCOOP_TRASH           =  9,
        PHASE_SCOOP_RETRACT         = 10,
    };

    TaskPhase phase;

    ros::NodeHandle nh_;

    // Publishers + Subscribers
    ros::Subscriber r_sub_, r_sub_ft_, l_sub_, l_sub_ft_;
    ros::Publisher  r_pub_, r_pub_ft_, r_pub_jstiff_, l_pub_, l_pub_jstiff, l_pub_ft_, ro_pub_, vo_pub_, vo_l_pub_, vo_r_pub_;
    string right_robot_frame, left_robot_frame;

    // Right/Left EE states/cmds/topics
    tf::Pose r_ee_pose, r_curr_ee_pose, r_des_ee_pose, l_ee_pose, l_curr_ee_pose, l_des_ee_pose;
    Eigen::VectorXd  r_curr_ee_ft, l_curr_ee_ft;
    string r_base_path, l_base_path, r_topic_ns, l_topic_ns;
    string model_base_path;
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

    // =====================================================
    //          FORCE control
    // =====================================================

    // Create messages for sending force commands
    geometry_msgs::WrenchStamped msg_ft;
    geometry_msgs::PoseStamped msg_pose;
    bool bWaitForForces_left_arm;
    bool bWaitForForces_right_arm; 

    // >>>>  LEFT ARM <<<<
    bool bUseForce_l_arm;               // True if force control should be applied
    bool bEnableForceModel_l_arm;       // True if force should be model based
    bool bForceModelInitialized_l_arm;  // True if GMR was initialized successfully
    bool bBypassForceModel_l_arm;       // True if a constant force should be used regardless of the model estimate
    bool bEndInContact_l_arm;           // True if after a reaching movement the end effector should be in contact with the environment

    double max_task_force_l_arm;        // in [N]
    double max_search_distance_l_arm;   // in cm
    double max_vertical_speed_l_arm;    // max speed to use when going down to search for contact
    double max_contact_force_l_arm;     // max force to use to establish contact on an object
    int    search_axis_l_arm;           // the direction in which to establish contact

    GMR *mForceModel_l_arm;

    // >>>> RIGHT ARM <<<<
    bool bUseForce_r_arm;               // True if force control should be applied
    bool bEnableForceModel_r_arm;       // True if force should be model based
    bool bForceModelInitialized_r_arm;  // True if GMR was initialized successfully
    bool bBypassForceModel_r_arm;       // True if a constant force should be used regardless of the model estimate
    bool bEndInContact_r_arm;           // True if after a reaching movement the end effector should be in contact with the environment

    double max_task_force_r_arm;        // in [N]
    double max_search_distance_r_arm;   // in cm
    double max_vertical_speed_r_arm;    // max speed to use when going down to search for contact
    double max_contact_force_r_arm;     // max force to use to establish contact on an object
    int    search_axis_r_arm;           // the direction in which to establish contact

    GMR *mForceModel_r_arm;

    double force_control_axis;          // the axis on which to perform force control X, Y, Z, RX, RY, RZ

    double desired_force;               // if constant force should be used
    double force_correction;            // position displacement to match to desired force
    double force_correction_max;        // maximum allowed position displacement to match to desired force
    double force_correction_delta;      // increment in the position displacement

    // FUNCTIONS USED FOR FORCE CONTROL

    void biasFtSensors();                                           // Zero the forces and torques after each reaching movement
    void send_and_wait_for_normal_force(double fz, int arm_id);     // Blocks until the desired force is achieved!
    bool find_object_by_contact(int arm_id, int search_dir, double search_distance, double search_speed, double thr_force); // Moves ee in the desired direction until contact is detected
    void initialize_force_model(std::string base_path, TaskPhase phase, int arm_id, string role);   // Initializes GMR for model based force profiles
    tf::Pose update_ee_pose_based_on_force(int arm_id, int ft_control_axis);                        // Only for actions that require continuous force
    tf::Pose remove_correction_due_to_force_from_trajectory(int arm_id, int ft_control_axis);       // When controlling with CDS

    // =====================================================
    //          CARTESIAN control
    // =====================================================

    // Filters for cartesian commands
    CDDynamics  *r_cdd_cart_filter;
    CDDynamics  *l_cdd_cart_filter;

    double r_pos_err, r_ori_err;
    double r_pos_gain, r_ori_gain, r_err_gain;

    double l_pos_err, l_ori_err;
    double l_pos_gain, l_ori_gain, l_err_gain;

    void initialize_cart_filter(double dt, double r_Wn, double l_Wn);
    void sync_cart_filter(const tf::Pose& r_ee_pose, const tf::Pose& l_ee_pose);
    void filter_arm_motion(tf::Pose& r_des_ee_pose, tf::Pose& l_des_ee_pose);

    bool bAdditionalTransforms; // True if the motion models were learned in a different RF than the actual EE frame. Than additional transforms are needed to get the correct motion
    void apply_task_specific_transformations(tf::Pose& left_final_target, tf::Pose& l_mNextRobotEEPose);

    // =====================================================
    //          SIMULATION AND VISUALIZATION
    // =====================================================

    // Simulation/execution variables
    volatile bool isOkay;
    bool initial_config, simulation, just_visualize;
    int tf_count;
    double reachingThreshold, orientationThreshold, model_dt, dt; //Defaults: [m],[rad],[s]
    double task_id;

    // Visualization variables for Bimanual DS Action
    visualization_msgs::Marker ro_marker;
    visualization_msgs::Marker vo_marker;
    visualization_msgs::Marker vo_arrow_left;
    visualization_msgs::Marker vo_arrow_right;



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

    // >>>>>>>>>>>> Autonomous behaviors <<<<<<<<<<<<<<<<<<<<<<<<<<<<

    // ACTION TYPE 1: Execute action from two independent learned models
    bool uncoupled_learned_model_execution(TaskPhase phase, CDSController::DynamicsType masterType, CDSController::DynamicsType slaveType,
                                           double reachingThreshold, double orientationThreshold, tf::Transform task_frame,
                                           tf::Transform right_att, tf::Transform left_att);

    // ACTION TYPE 2: Execute bimanual reach with virtual object dynamical system
    bool coordinated_bimanual_ds_execution(TaskPhase phase, tf::Transform task_frame, tf::Transform right_att, tf::Transform left_att, double dt);


    // ACTION TYPE 3: Execute bimanual reach with virtual object dynamical system
    bool coupled_learned_model_execution(TaskPhase phase, CDSController::DynamicsType r_masterType, CDSController::DynamicsType r_slaveType, CDSController::DynamicsType l_masterType, CDSController::DynamicsType l_slaveType,
                                         double reachingThreshold, double orientationThreshold,  tf::Transform task_frame,
                                         tf::Transform right_att, tf::Transform left_att);

    // ACTION TYPE 4: Go to Targets in Cartesian Space
    bool bimanual_goto_cart_execution(tf::Transform task_frame, tf::Transform right_att, tf::Transform left_att);

    // >>>>>>>>>>>> Collaborative Behaviors <<<<<<<<<<<<<<<<<<<<<<<<<<

    // Action Type 1: Robot Arm is passive

    // Action Type 2: Robot arm is active


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

    void publish_task_frames(tf::Pose& r_curr_ee_pose, tf::Pose& l_curr_ee_pose, tf::Transform& right_final_target,
                             tf::Transform& left_final_target, tf::Transform& task_frame);

public:

    //**************************//
    // ACTION SERVER FUNCTIONS  //
    //**************************//

    BimanualActionServer(std::string name);

    ~BimanualActionServer(void);

    void initialize();

    void executeCB(const bimanual_action_planners::PLAN2CTRLGoalConstPtr &goal);


};
