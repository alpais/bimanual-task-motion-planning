#include "bimanual_action_server.h"

BimanualActionServer::BimanualActionServer(std::string name) :
    as_(nh_, name, boost::bind(&BimanualActionServer::executeCB, this, _1), false),
    action_name_(name), tf_count(0), reachingThreshold (0.01), orientationThreshold (0.1),
    model_dt (0.002), dt (0.002), initial_config(true)
{
    r_curr_ee_ft.resize(6);
    l_curr_ee_ft.resize(6);
    as_.start();

}


BimanualActionServer::~BimanualActionServer(void)
{
}


void BimanualActionServer::initialize() {

    bDisplayDebugInfo = false;

    bBypassOri = false;    // Compute the orientation using CDDynamics
    bFilterOri = false;    // Smooth the ori
    bIgnoreOri = false;   // Completly discard the orientation

    ros::NodeHandle _nh("~");

    // Read Parameters from Launch File
    _nh.getParam("right_robot_frame", right_robot_frame);
    _nh.getParam("left_robot_frame", left_robot_frame);
    _nh.getParam("model_base_path", model_base_path);
    _nh.getParam("simulation", simulation);
    _nh.getParam("just_visualize", just_visualize);
    _nh.getParam("model_dt", model_dt);
    _nh.getParam("reachingThreshold", reachingThreshold);
    _nh.getParam("orientationThreshold", orientationThreshold);
    _nh.getParam("r_topic_ns", r_topic_ns);
    _nh.getParam("l_topic_ns", l_topic_ns);
    _nh.getParam("wait_for_force_right", bWaitForForces_right_arm);
    _nh.getParam("wait_for_force_left", bWaitForForces_left_arm);
    _nh.getParam("task_id", task_id);
    _nh.getParam("enable_force_model_l_arm", bEnableForceModel_l_arm);
    _nh.getParam("enable_force_model_r_arm", bEnableForceModel_r_arm);

    if(!_nh.getParam("wait_for_force_right", bWaitForForces_right_arm) && task_id == PEELING_TASK_ID) {
        ROS_INFO_STREAM("Set the Waiting for forces flag");
        bWaitForForces_right_arm = true;
    }

    if(!_nh.getParam("wait_for_force_left", bWaitForForces_left_arm)) {
        ROS_INFO_STREAM("Set the Waiting for forces flag");
        bWaitForForces_left_arm = true;
    }

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

    std::stringstream vis_ss_bowl_pose, vis_ss_wrist_pose;
    vis_ss_bowl_pose << "Bowl_Frame/pose";
    vis_ss_wrist_pose << "Human_Wrist/pose";

    VISION_BOWL_POSE_TOPIC = vis_ss_bowl_pose.str();
    VISION_WRIST_POSE_TOPIC = vis_ss_wrist_pose.str();


    // Right Arm
    R_EE_STATE_POSE_TOPIC = r_ss_state_pose.str();
    R_EE_STATE_FT_TOPIC	  = r_ss_state_ft.str();
    R_STATE_STIFF_TOPIC   = r_ss_state_stiff.str();

    R_EE_CMD_POSE_TOPIC	  = r_ss_cmd_pose.str();
    R_EE_CMD_FT_TOPIC	  = r_ss_cmd_ft.str();
    R_CMD_STIFF_TOPIC     = r_ss_cmd_stiff.str();

    // Left Arm
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
    h_action_state_sub_ = nh_.subscribe<std_msgs::Bool>("state_estimator/action_state", 1, &BimanualActionServer::h_currentActionStateCallback, this);

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


    // Service Clients for FT/Sensor Biasing
    hand_ft_client = nh_.serviceClient<netft_rdt_driver::String_cmd>("/hand/ft_sensor/bias_cmd");
    tool_ft_client = nh_.serviceClient<netft_rdt_driver::String_cmd>("/tool/ft_sensor/bias_cmd");


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

    // Getting vision objects
    try {
        listener.waitForTransform("/world_frame", "Bowl_Frame/base_link", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("/world_frame", "Bowl_Frame/base_link",  ros::Time(0), bowl_in_base_transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }
    ROS_INFO_STREAM("Bowl in base: " << bowl_in_base_transform.getOrigin().x() << " " << bowl_in_base_transform.getOrigin().y() << " " << bowl_in_base_transform.getOrigin().z());

    try {
        listener.waitForTransform("/world_frame", "Human_Wrist/base_link", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("/world_frame", "Human_Wrist/base_link", ros::Time(0), wrist_in_base_transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }
    try {
        listener.waitForTransform("Bowl_Frame/base_link", "/wrist_in_base_transform", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("Bowl_Frame/base_link", "/wrist_in_base_transform",  ros::Time(0), wrist_in_att_transform);
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


void BimanualActionServer::executeCB(const bimanual_action_planners::PLAN2CTRLGoalConstPtr &goal)
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
    task_frame.setRotation(tf::Quaternion(goal->task_frame.rotation.x,goal->task_frame.rotation.y, goal->task_frame.rotation.z,
                                          goal->task_frame.rotation.w));
    task_frame.setOrigin(tf::Vector3(goal->task_frame.translation.x, goal->task_frame.translation.y, goal->task_frame.translation.z));

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

    task_id = SCOOPING_TASK_ID; // Apparently this value gets overwritten at each call of the action server. First time it works, second time it becomes 1

    // filter gains
    double r_filter_gain_Wn = 25;
    double l_filter_gain_Wn = 10;

    // Default values for CDS control right arm
    CDSController::DynamicsType r_masterType = CDSController::MODEL_DYNAMICS;
    CDSController::DynamicsType r_slaveType = CDSController::UTHETA;

    // Default values for CDS control left arm
    CDSController::DynamicsType l_masterType = CDSController::MODEL_DYNAMICS;
    CDSController::DynamicsType l_slaveType = CDSController::UTHETA;

    bool bActionTypeReach = false;
    bAdditionalTransforms = false;

    // TODO: the Action-specific parameters should be read from yaml files


    if (task_id == PEELING_TASK_ID){
        if(goal->action_name == "phase0"){
            phase = PHASE_INIT_REACH;

            bEndInContact_l_arm = false;
            bEndInContact_r_arm = true;

            bEnableForceModel_l_arm = false;
            bEnableForceModel_r_arm = false;

            search_axis_r_arm = SEARCH_DIR_Z;
            max_task_force_r_arm        = 15;
            max_search_distance_r_arm   = 0.07;
            max_vertical_speed_r_arm    = 0.01;
            max_contact_force_r_arm     = 10;

            bUseForce_l_arm = false;
            bUseForce_r_arm = false;

            r_pos_gain = 1;
            r_ori_gain = 0.5;
            r_err_gain = 1;

            l_pos_gain = 1;
            l_ori_gain = 1;
            l_err_gain = 1;

            r_filter_gain_Wn = 25;
            l_filter_gain_Wn = 10;

            bActionTypeReach = true;
            bAdditionalTransforms = false;

        }
        else if(goal->action_name   == "phase1") {
            phase = PHASE_REACH_TO_PEEL;
            bEndInContact_l_arm = true;
            bEndInContact_r_arm = false;

            search_axis_l_arm = SEARCH_DIR_Z;
            max_task_force_l_arm        = 15;
            max_search_distance_l_arm   = 0.07;
            max_vertical_speed_l_arm    = 0.01;
            max_contact_force_l_arm     = 8;

            bUseForce_l_arm = false;
            bUseForce_r_arm = false;

            bEnableForceModel_l_arm = false;
            bEnableForceModel_r_arm = false;

            r_pos_gain = 1;
            r_ori_gain = 0.5;
            r_err_gain = 1;

            l_pos_gain = 1;
            l_ori_gain = 1;
            l_err_gain = 1;

            r_masterType = CDSController::MODEL_DYNAMICS;
            r_slaveType = CDSController::UTHETA;

            l_masterType = CDSController::MODEL_DYNAMICS;
            l_slaveType = CDSController::UTHETA;

            r_filter_gain_Wn = 25;
            l_filter_gain_Wn = 10;

            bActionTypeReach = true;
            bAdditionalTransforms = true;


        } else if(goal->action_name == "phase2") {

            phase = PHASE_PEEL;
            force_control_axis = FT_CTRL_AXIS_Z;
            bUseForce_l_arm = true;
            bBypassForceModel_l_arm = true;

            force_correction            = 0;
            desired_force               = 8;
            force_correction_max        = 0.05;    // 5cm
            force_correction_delta      = 0.001; // 1 mm

            bUseForce_l_arm = true;
            bUseForce_r_arm = false;

            bEnableForceModel_l_arm = false;
            bEnableForceModel_r_arm = false;

            r_pos_gain = 1;
            r_ori_gain = 0.5;
            r_err_gain = 1;

            l_pos_gain = 1;
            l_ori_gain = 1;
            l_err_gain = 1;

            r_masterType = CDSController::MODEL_DYNAMICS;
            r_slaveType = CDSController::UTHETA;

            l_masterType = CDSController::MODEL_DYNAMICS;
            l_slaveType = CDSController::UTHETA;

            r_filter_gain_Wn = 25;
            l_filter_gain_Wn = 10;

            bActionTypeReach = false;
            bAdditionalTransforms = false;

        } else if(goal->action_name == "phase3") {
            phase = PHASE_ROTATE;

            bEndInContact_l_arm = false;
            bEndInContact_r_arm = false;

            bUseForce_l_arm = false;
            bUseForce_r_arm = false;

            r_pos_gain = 1;
            r_ori_gain = 0.5;
            r_err_gain = 1;

            l_pos_gain = 1;
            l_ori_gain = 1;
            l_err_gain = 1;

            r_masterType = CDSController::MODEL_DYNAMICS;
            r_slaveType = CDSController::UTHETA;

            l_masterType = CDSController::MODEL_DYNAMICS;
            l_slaveType = CDSController::UTHETA;

            r_filter_gain_Wn = 25;
            l_filter_gain_Wn = 10;

            bActionTypeReach = false;
            bAdditionalTransforms = false;

        } else if(goal->action_name == "phase4") {
            phase = PHASE_RETRACT;

            bEndInContact_l_arm = false;
            bEndInContact_r_arm = false;

            bUseForce_l_arm = false;
            bUseForce_r_arm = false;

            bEnableForceModel_l_arm = false;
            bEnableForceModel_r_arm = false;

            r_pos_gain = 1;
            r_ori_gain = 0.5;
            r_err_gain = 1;

            l_pos_gain = 1;
            l_ori_gain = 1;
            l_err_gain = 1;

            r_masterType = CDSController::MODEL_DYNAMICS;
            r_slaveType = CDSController::UTHETA;

            l_masterType = CDSController::MODEL_DYNAMICS;
            l_slaveType = CDSController::UTHETA;

            r_filter_gain_Wn = 25;
            l_filter_gain_Wn = 10;

            bActionTypeReach = false;
            bAdditionalTransforms = false;

        } else {
            ROS_ERROR_STREAM("Unidentified action name "<<goal->action_name.c_str());
            result_.success = 0;
            as_.setAborted(result_);
            return;
        }
    }
    else if (task_id == SCOOPING_TASK_ID) {
        if(goal->action_name == "phase0"){
            phase = PHASE_SCOOP_INIT_REACH;

            bUseForce_l_arm = false;
            bUseForce_r_arm = false;

            bEndInContact_l_arm = false;
            bEndInContact_r_arm = false;

            bEnableForceModel_l_arm = false;
            bEnableForceModel_r_arm = false;

            r_pos_gain = 1;
            r_ori_gain = 0.5;
            r_err_gain = 1;

            l_pos_gain = 1;
            l_ori_gain = 1;
            l_err_gain = 1;

            r_avg_jstiff = INTERACTION_STIFFNESS;
            l_avg_jstiff = INTERACTION_STIFFNESS;

            r_masterType = CDSController::MODEL_DYNAMICS;
            r_slaveType = CDSController::UTHETA;

            l_masterType = CDSController::MODEL_DYNAMICS;
            l_slaveType = CDSController::UTHETA;

            r_filter_gain_Wn = 25;
            l_filter_gain_Wn = 10;

            bActionTypeReach = true;
            bAdditionalTransforms = false;
        }
        else if(goal->action_name   == "phase1") {
            phase = PHASE_SCOOP_REACH_TO_SCOOP;

            bEndInContact_l_arm = true;
            bEndInContact_r_arm = false;

            search_axis_l_arm = SEARCH_DIR_Z;
            max_task_force_l_arm        = 5;
            max_search_distance_l_arm   = 0.08;
            max_vertical_speed_l_arm    = 0.01;
            max_contact_force_l_arm     = 12;

            bUseForce_l_arm = false;
            bUseForce_r_arm = false;

            bEnableForceModel_l_arm = false;
            bEnableForceModel_r_arm = false;

            r_pos_gain = 0.5;
            r_ori_gain = 1.5;
            r_err_gain = 1;

            l_pos_gain = 1;
            l_ori_gain = 1;
            l_err_gain = 1;

            r_masterType = CDSController::LINEAR_DYNAMICS;
            r_slaveType = CDSController::UTHETA;

            l_masterType = CDSController::LINEAR_DYNAMICS;
            l_slaveType = CDSController::NO_DYNAMICS;

            r_filter_gain_Wn = 25;
            l_filter_gain_Wn = 10;

            bActionTypeReach = true;
            bAdditionalTransforms = false;

            r_avg_jstiff = INTERACTION_STIFFNESS;
            l_avg_jstiff = INTERACTION_STIFFNESS;

        } else if(goal->action_name == "phase2") {

            phase = PHASE_SCOOP_SCOOP;

            bUseForce_l_arm = true;
            bBypassForceModel_l_arm = true;

            force_control_axis = FT_CTRL_AXIS_RZ;

            force_correction = 0;
            desired_force = 0.12;                // torque around Z
            force_correction_max = 0.01;        // 1cm
            force_correction_delta = 0.001;     // 1 mm
            bUseForce_r_arm = false;

            bEndInContact_l_arm = false;
            bEndInContact_r_arm = false;

            bUseForce_l_arm = true;
            bUseForce_r_arm = false;

            bEnableForceModel_l_arm = false;
            bEnableForceModel_r_arm = false;

            r_pos_gain = 1;
            r_ori_gain = 0.5;
            r_err_gain = 1;

            l_pos_gain = 1;
            l_ori_gain = 1.5;
            l_err_gain = 1.5;

            r_avg_jstiff = TASK_STIFFNESS;
            l_avg_jstiff = TASK_STIFFNESS;

            r_masterType = CDSController::MODEL_DYNAMICS;
            r_slaveType = CDSController::UTHETA;

            l_masterType = CDSController::MODEL_DYNAMICS;
            l_slaveType = CDSController::UTHETA;

            r_filter_gain_Wn = 25;
            l_filter_gain_Wn = 10;

            bActionTypeReach = false;
            bAdditionalTransforms = false;

        } else if(goal->action_name == "phase3") {
            phase = PHASE_SCOOP_DEPART;

            bEndInContact_l_arm = false;
            bEndInContact_r_arm = false;

            bUseForce_l_arm = false;
            bUseForce_r_arm = false;

            bEnableForceModel_l_arm = false;
            bEnableForceModel_r_arm = false;

            r_pos_gain = 1;
            r_ori_gain = 0.5;
            r_err_gain = 2.5;

            l_pos_gain = 1;
            l_ori_gain = 1;
            l_err_gain = 1.5;

            r_avg_jstiff = INTERACTION_STIFFNESS;
            l_avg_jstiff = INTERACTION_STIFFNESS;

            r_masterType = CDSController::LINEAR_DYNAMICS;
            r_slaveType = CDSController::UTHETA;

            l_masterType = CDSController::LINEAR_DYNAMICS;
            l_slaveType = CDSController::UTHETA;

            r_filter_gain_Wn = 25;
            l_filter_gain_Wn = 10;

            bActionTypeReach = true;
            bAdditionalTransforms = false;

        } else if(goal->action_name == "phase4") {
            phase = PHASE_SCOOP_TRASH;

            bEndInContact_l_arm = false;
            bEndInContact_r_arm = false;

            bUseForce_l_arm = false;
            bUseForce_r_arm = false;

            bEnableForceModel_l_arm = false;
            bEnableForceModel_r_arm = false;

            r_pos_gain = 1;
            r_ori_gain = 0.5;
            r_err_gain = 1.5;

            l_pos_gain = 1;
            l_ori_gain = 1.5;
            l_err_gain = 2;

            r_masterType = CDSController::LINEAR_DYNAMICS;
            r_slaveType = CDSController::UTHETA;

            l_masterType = CDSController::LINEAR_DYNAMICS;
            l_slaveType = CDSController::NO_DYNAMICS;

            r_filter_gain_Wn = 55;
            l_filter_gain_Wn = 25;

            bActionTypeReach = true;
            bAdditionalTransforms = false;

        } else if(goal->action_name == "phase5") {
            phase = PHASE_SCOOP_RETRACT;

            bEndInContact_l_arm = false;
            bEndInContact_r_arm = false;

            bUseForce_l_arm = false;
            bUseForce_r_arm = false;

            bEnableForceModel_l_arm = false;
            bEnableForceModel_r_arm = false;

            r_pos_gain = 1;
            r_ori_gain = 0.5;
            r_err_gain = 1;

            l_pos_gain = 1;
            l_ori_gain = 1;
            l_err_gain = 1;

            r_avg_jstiff = INTERACTION_STIFFNESS;
            l_avg_jstiff = INTERACTION_STIFFNESS;

            r_masterType = CDSController::MODEL_DYNAMICS;
            r_slaveType = CDSController::UTHETA;

            l_masterType = CDSController::MODEL_DYNAMICS;
            l_slaveType = CDSController::UTHETA;

            r_filter_gain_Wn = 25;
            l_filter_gain_Wn = 10;

            bActionTypeReach = true;
            bAdditionalTransforms = false;

        } else {
            ROS_ERROR_STREAM("Unidentified action name "<<goal->action_name.c_str());
            result_.success = 0;
            as_.setAborted(result_);
            return;
        }

    } else {
        ROS_INFO("INVALID TASK");
        exit(1);
    }


    initialize_cart_filter(dt, r_filter_gain_Wn, l_filter_gain_Wn);
    sync_cart_filter(r_ee_pose, l_ee_pose);

    // For each reaching action we bias the FT sensors
    if (bActionTypeReach)
        biasFtSensors();

    if (bEnableForceModel_l_arm && !bBypassForceModel_l_arm){
        ROS_INFO("Attempting to initialize force model for left arm");
        initialize_force_model(model_base_path, phase, L_ARM_ID, L_ARM_ROLE);
    }

    if (bEnableForceModel_r_arm && !bBypassForceModel_r_arm){
        ROS_INFO("Attempting to initialize force model for right arm");
        initialize_force_model(model_base_path, phase, R_ARM_ID, R_ARM_ROLE);
    }

    //---> ACTION TYPE 1: Use two independent learned models to execute the action
    if(goal->action_type=="UNCOUPLED_LEARNED_MODEL"){
        CDSController::DynamicsType masterType = CDSController::MODEL_DYNAMICS;
        CDSController::DynamicsType slaveType = CDSController::UTHETA;
        // Execute action from *uncoupled* learned action model
        success = uncoupled_learned_model_execution(phase, masterType, slaveType, reachingThreshold, orientationThreshold,
                                                    task_frame, right_att, left_att);
    }

    //---> ACTION TYPE 2: Use the virtual object dynamical system to execute a bimanual reach
    if(goal->action_type=="BIMANUAL_REACH")
        success = coordinated_bimanual_ds_execution(phase, task_frame, right_att, left_att, DT);


    //---> ACTION TYPE 3: Use two coupled learned models to execute the action
    if(goal->action_type=="COUPLED_LEARNED_MODEL"){
        // Execute action from *coupled* learned action model
        success = coupled_learned_model_execution(phase, r_masterType, r_slaveType, l_masterType, l_slaveType, reachingThreshold, orientationThreshold,
                                                  task_frame, right_att, left_att);
    }

    //---> ACTION TYPE 4: Use coupled learned models to execute the action
    if(goal->action_type=="BIMANUAL_GOTO_CART")
        success = bimanual_goto_cart_execution(task_frame, right_att, left_att);
    result_.success = success;

    //---> ACTION TYPE 5: Collaborative - Robot arm is passive
    if(goal->action_type=="COLLABORATIVE_PASSIVE"){
        // Execute model based action in collab
        bEnableCollaborativeMode = true;
        success = collab_passive_model_execution(phase, task_frame, right_att, DT, r_masterType, r_slaveType, reachingThreshold, orientationThreshold, left_att);
    }

    if(success)
    {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        as_.setSucceeded(result_);
    } else {
        ROS_INFO("%s: Failed", action_name_.c_str());
        as_.setAborted(result_);
    }

}

