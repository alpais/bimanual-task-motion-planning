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

    if(!_nh.getParam("wait_for_force_right", bWaitForForces_right_arm) && task_id == PEELING_TASK_ID) {
        ROS_INFO_STREAM("Set the Waiting for forces flag");
        bWaitForForces_right_arm = true;
    }

    if(!_nh.getParam("wait_for_force_left", bWaitForForces_left_arm)) {
        ROS_INFO_STREAM("Set the Waiting for forces flag");
        bWaitForForces_left_arm = true;
    }

    std::stringstream r_ss_state_pose, r_ss_state_ft, r_ss_cmd_pose, r_ss_cmd_ft;
    r_ss_state_pose << "/" << r_topic_ns << "/joint_to_cart/est_ee_pose";
    r_ss_state_ft   << "/hand/ft_sensor/netft_data";
    r_ss_cmd_pose   << "/" << r_topic_ns << "/cart_to_joint/des_ee_pose";
    r_ss_cmd_ft     << "/" << r_topic_ns << "/cart_to_joint/des_ee_ft";

    R_EE_STATE_POSE_TOPIC = r_ss_state_pose.str();
    R_EE_STATE_FT_TOPIC	  = r_ss_state_ft.str();
    R_EE_CMD_POSE_TOPIC	  = r_ss_cmd_pose.str();
    R_EE_CMD_FT_TOPIC	  = r_ss_cmd_ft.str();


    // ROS TOPICS for right arm controllers
    r_sub_    = nh_.subscribe<geometry_msgs::PoseStamped>(R_EE_STATE_POSE_TOPIC, 1, &BimanualActionServer::r_eeStateCallback, this);
    r_pub_    = nh_.advertise<geometry_msgs::PoseStamped>(R_EE_CMD_POSE_TOPIC, 1);

    r_sub_ft_ = nh_.subscribe<geometry_msgs::WrenchStamped>(R_EE_STATE_FT_TOPIC, 1, &BimanualActionServer::r_ftStateCallback, this);
    r_pub_ft_ = nh_.advertise<geometry_msgs::WrenchStamped>(R_EE_CMD_FT_TOPIC, 1);



    std::stringstream l_ss_state_pose, l_ss_state_ft, l_ss_cmd_pose, l_ss_cmd_ft;
    l_ss_state_pose << "/" << l_topic_ns << "/joint_to_cart/est_ee_pose";
    l_ss_state_ft   << "/tool/ft_sensor/netft_data";
    l_ss_cmd_pose   << "/" << l_topic_ns << "/cart_to_joint/des_ee_pose";
    l_ss_cmd_ft     << "/" << l_topic_ns << "/cart_to_joint/des_ee_ft";

    L_EE_STATE_POSE_TOPIC = l_ss_state_pose.str();
    L_EE_STATE_FT_TOPIC	  = l_ss_state_ft.str();
    L_EE_CMD_POSE_TOPIC	  = l_ss_cmd_pose.str();
    L_EE_CMD_FT_TOPIC	  = l_ss_cmd_ft.str();


    r_curr_ee_ft.resize(6);
    l_curr_ee_ft.resize(6);

    // ROS TOPICS for left arm controllers
    l_sub_    = nh_.subscribe<geometry_msgs::PoseStamped>(L_EE_STATE_POSE_TOPIC, 1, &BimanualActionServer::l_eeStateCallback, this);
    l_sub_ft_ = nh_.subscribe<geometry_msgs::WrenchStamped>(L_EE_STATE_FT_TOPIC, 1, &BimanualActionServer::l_ftStateCallback, this);
    l_pub_    = nh_.advertise<geometry_msgs::PoseStamped>(L_EE_CMD_POSE_TOPIC, 1);
    l_pub_ft_ = nh_.advertise<geometry_msgs::WrenchStamped>(L_EE_CMD_FT_TOPIC, 1);


    ROS_INFO_STREAM("Right (BHand-Zucchini) FT Sensor: " << R_EE_STATE_FT_TOPIC);
    ROS_INFO_STREAM("Left (Peel-Tool) FT Sensor: " << L_EE_STATE_FT_TOPIC);


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

    TaskPhase phase;
    if (task_id == PEELING_TASK_ID){
        if(goal->action_name == "phase0"){
            phase = PHASE_INIT_REACH;
        }
        else if(goal->action_name   == "phase1") {
            phase = PHASE_REACH_TO_PEEL;
        } else if(goal->action_name == "phase2") {
            phase = PHASE_PEEL;
        } else if(goal->action_name == "phase3") {
            phase = PHASE_ROTATE;
        } else if(goal->action_name == "phase4") {
            phase = PHASE_RETRACT;
        } else {
            ROS_ERROR_STREAM("Unidentified action name "<<goal->action_name.c_str());
            result_.success = 0;
            as_.setAborted(result_);
            return;
        }
    }
    else{
        if(goal->action_name == "phase0"){
            phase = PHASE_SCOOP_INIT_REACH;
        }
        else if(goal->action_name   == "phase1") {
            phase = PHASE_SCOOP_REACH_TO_SCOOP;
        } else if(goal->action_name == "phase2") {
            phase = PHASE_SCOOP_SCOOP;
        } else if(goal->action_name == "phase3") {
            phase = PHASE_SCOOP_DEPART;
        } else if(goal->action_name == "phase4") {
            phase = PHASE_SCOOP_TRASH;
        } else if(goal->action_name == "phase5") {
            phase = PHASE_SCOOP_RETRACT;
        } else {
            ROS_ERROR_STREAM("Unidentified action name "<<goal->action_name.c_str());
            result_.success = 0;
            as_.setAborted(result_);
            return;
        }

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
        CDSController::DynamicsType masterType = CDSController::MODEL_DYNAMICS;
        CDSController::DynamicsType slaveType = CDSController::NO_DYNAMICS;
        // Execute action from *coupled* learned action model
        success = coupled_learned_model_execution(phase, masterType, slaveType, reachingThreshold, orientationThreshold,
                                                  task_frame, right_att, left_att);
    }

    //---> ACTION TYPE 4: Use coupled learned models to execute the action
    if(goal->action_type=="BIMANUAL_GOTO_CART")
        success = bimanual_goto_cart_execution(task_frame, right_att, left_att);
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

