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

    bDisplayDebugInfo = false;      // True to display position and orientation errors

    bBypassOri = false;     // Compute the orientation using CDDynamics
    bFilterOri = false;     // Smooth the ori
    bIgnoreOri = false;     // Completly discard the orientation

    get_parameters();

    initialize_ros_publishers_and_subscribers();

    get_initial_transforms();

    // Service Clients for FT/Sensor Biasing
    hand_ft_client = nh_.serviceClient<netft_rdt_driver::String_cmd>("/hand/ft_sensor/bias_cmd");
    tool_ft_client = nh_.serviceClient<netft_rdt_driver::String_cmd>("/tool/ft_sensor/bias_cmd");

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

    bActionTypeReach = false;
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

    //---> ACTION TYPE 6: Collaborative - Robot arm is active
    if(goal->action_type=="COLLABORATIVE_ACTIVE"){
        // Execute model based action in collab
        bEnableCollaborativeMode = true;
        success = collab_active_model_execution(phase, task_frame, left_att, DT, l_masterType, l_slaveType, reachingThreshold, orientationThreshold, right_att);
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

