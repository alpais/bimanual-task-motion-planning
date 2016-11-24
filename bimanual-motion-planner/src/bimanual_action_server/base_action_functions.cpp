#include "bimanual_action_server.h"


bool BimanualActionServer::initialize_coupling_model(std::string base_path, TaskPhase phase, tf::Vector3 master_dim, tf::Vector3 slave_dim){


    // Coupling models are only for the reaching phases, not for force control phases

    char sCoupling[256];
    sprintf(sCoupling, "%s/Phase%d/arm_cplGMM.txt", base_path.c_str(), phase);

    int inp; inp = 0;
    //    master_dim.getX();


}


void BimanualActionServer::initialize_force_model(std::string base_path, TaskPhase phase, int arm_id, std::string role){

    string arm;
    if (arm_id == R_ARM_ID)
        arm = "Right";
    if (arm_id == L_ARM_ID)
        arm = "Left";


    if (arm_id == L_ARM_ID){
        if (bForceModelInitialized_l_arm){
            delete mForceModel_l_arm;
            bForceModelInitialized_l_arm = false;
        }
        else {
            char sForce[256];
            sprintf(sForce, "%s/Phase%d/%s/%s_forGMM.txt", base_path.c_str(), phase, arm.c_str(), role.c_str());

            std::vector<int> in_dim;  in_dim.resize(1);  in_dim[0] = 0;
            std::vector<int> out_dim; out_dim.resize(1); out_dim[0] = 1;

            mForceModel_l_arm = new GMR(sForce);
            mForceModel_l_arm->initGMR(in_dim, out_dim);

            if (!mForceModel_l_arm->getIsInit()){
                ROS_INFO_STREAM("ERROR: Could not initialize force model for the " << arm << " arm");
                exit(1);
            }
            else {
                bForceModelInitialized_l_arm = true;
                mForceModel_l_arm->printInfo();
            }
        }
    }
    else if (arm_id == R_ARM_ID){
        if (bForceModelInitialized_r_arm){
            delete mForceModel_r_arm;
            bForceModelInitialized_r_arm = false;
        }
        else {
            char sForce[256];
            sprintf(sForce, "%s/Phase%d/%s/%s_forGMM.txt", base_path.c_str(), phase, arm.c_str(), role.c_str());

            std::vector<int> in_dim;  in_dim.resize(1);  in_dim[0] = 0;
            std::vector<int> out_dim; out_dim.resize(1); out_dim[0] = 1;

            mForceModel_r_arm = new GMR(sForce);
            mForceModel_r_arm->initGMR(in_dim, out_dim);

            if (!mForceModel_r_arm->getIsInit()){
                ROS_INFO_STREAM("ERROR: Could not initialize force model for the " << arm << " arm");
                exit(1);
            }
            else {
                bForceModelInitialized_r_arm = true;
                mForceModel_r_arm->printInfo();
            }
        }

    } else {
        ROS_ERROR("Arm ID invalid. Could not initialize force from model");
        exit(1);
    }

}



void BimanualActionServer::initialize_stiffness_model(std::string base_path, TaskPhase phase, int arm_id, std::string role){

    string arm;
    if (arm_id == R_ARM_ID)
        arm = "Right";
    if (arm_id == L_ARM_ID)
        arm = "Left";


    if (arm_id == L_ARM_ID){
        if (bStiffModelInitialized_l_arm){
            delete mStiffModel_l_arm;
            bStiffModelInitialized_l_arm = false;
        }
        else {
            char sStiff[256];
            sprintf(sStiff, "%s/Phase%d/%s/%s_stiffGMM.txt", base_path.c_str(), phase, arm.c_str(), role.c_str());

            std::vector<int> in_dim;  in_dim.resize(1);  in_dim[0] = 0;
            std::vector<int> out_dim; out_dim.resize(1); out_dim[0] = 1;

            mStiffModel_l_arm = new GMR(sStiff);
            mStiffModel_l_arm->initGMR(in_dim, out_dim);

            if (!mStiffModel_l_arm->getIsInit()){
                ROS_INFO_STREAM("ERROR: Could not initialize stiffness model for the " << arm << " arm");
                exit(1);
            }
            else {
                bStiffModelInitialized_l_arm = true;
                mStiffModel_l_arm->printInfo();
            }
        }
    }
    else if (arm_id == R_ARM_ID){
        if (bStiffModelInitialized_r_arm){
            delete mStiffModel_r_arm;
            bStiffModelInitialized_r_arm = false;
        }
        else {
            char sStiff[256];
            sprintf(sStiff, "%s/Phase%d/%s/%s_stiffGMM.txt", base_path.c_str(), phase, arm.c_str(), role.c_str());

            std::vector<int> in_dim;  in_dim.resize(1);  in_dim[0] = 0;
            std::vector<int> out_dim; out_dim.resize(1); out_dim[0] = 1;

            mStiffModel_r_arm = new GMR(sStiff);
            mStiffModel_r_arm->initGMR(in_dim, out_dim);

            if (!mStiffModel_r_arm->getIsInit()){
                ROS_INFO_STREAM("ERROR: Could not initialize stiffness model for the " << arm << " arm");
                exit(1);
            }
            else {
                bStiffModelInitialized_r_arm = true;
                mStiffModel_r_arm->printInfo();
            }
        }

    } else {
        ROS_ERROR("Arm ID invalid. Could not initialize force from model");
        exit(1);
    }

}


//void initialize_cart_filter(double dt, double r_Wn, double l_Wn){
void BimanualActionServer::initialize_cart_filter(double dt, double r_Wn, double l_Wn){

    // Filter for right arm
    r_cdd_cart_filter = new CDDynamics(7, dt, r_Wn);
    MathLib::Vector r_vel_lim_cart(7);
    r_vel_lim_cart = DEG2RAD(60);
    r_cdd_cart_filter->SetVelocityLimits(r_vel_lim_cart);
    r_cdd_cart_filter->SetWn(r_Wn);

    // Filter for left arm
    l_cdd_cart_filter = new CDDynamics(7, dt, l_Wn);
    MathLib::Vector l_vel_lim_cart(7);
    l_vel_lim_cart = DEG2RAD(60);
    l_cdd_cart_filter->SetVelocityLimits(l_vel_lim_cart);
    l_cdd_cart_filter->SetWn(l_Wn);


}

void BimanualActionServer::sync_cart_filter(const tf::Pose& r_ee_pose, const tf::Pose& l_ee_pose){

    // synchronize Cartesian motion for the right arm
    MathLib::Vector r_init_state; r_init_state.Resize(7, false);
    r_init_state(0) = r_ee_pose.getOrigin().getX();
    r_init_state(1) = r_ee_pose.getOrigin().getY();
    r_init_state(2) = r_ee_pose.getOrigin().getZ();
    r_init_state(3) = r_ee_pose.getRotation().getX();
    r_init_state(4) = r_ee_pose.getRotation().getY();
    r_init_state(5) = r_ee_pose.getRotation().getZ();
    r_init_state(6) = r_ee_pose.getRotation().getW();
    r_cdd_cart_filter->SetState(r_init_state);

    // synchronize Cartesian motion for the left arm
    MathLib::Vector l_init_state; l_init_state.Resize(7, false);
    l_init_state(0) = l_ee_pose.getOrigin().getX();
    l_init_state(1) = l_ee_pose.getOrigin().getY();
    l_init_state(2) = l_ee_pose.getOrigin().getZ();
    l_init_state(3) = l_ee_pose.getRotation().getX();
    l_init_state(4) = l_ee_pose.getRotation().getY();
    l_init_state(5) = l_ee_pose.getRotation().getZ();
    l_init_state(6) = l_ee_pose.getRotation().getW();
    l_cdd_cart_filter->SetState(l_init_state);

    ROS_INFO("Synchronized Cartesian motion");

}

void BimanualActionServer::filter_arm_motion(tf::Pose& r_des_ee_pose, tf::Pose& l_des_ee_pose){

    // Filter right arm motion
    MathLib::Vector r_next_target; r_next_target.Resize(7, false);
    r_next_target(0) = r_des_ee_pose.getOrigin().getX();
    r_next_target(1) = r_des_ee_pose.getOrigin().getY();
    r_next_target(2) = r_des_ee_pose.getOrigin().getZ();
    r_next_target(3) = r_des_ee_pose.getRotation().getX();
    r_next_target(4) = r_des_ee_pose.getRotation().getY();
    r_next_target(5) = r_des_ee_pose.getRotation().getZ();
    r_next_target(6) = r_des_ee_pose.getRotation().getW();

    r_cdd_cart_filter->SetTarget(r_next_target);
    r_cdd_cart_filter->Update();
    r_cdd_cart_filter->GetState(r_next_target);

    r_des_ee_pose.setOrigin(tf::Vector3(r_next_target(0), r_next_target(1), r_next_target(2)));
    r_des_ee_pose.setRotation(tf::Quaternion(r_next_target(3), r_next_target(4), r_next_target(5), r_next_target(6)));;

    // Filter left arm motion
    MathLib::Vector l_next_target; l_next_target.Resize(7, false);
    l_next_target(0) = l_des_ee_pose.getOrigin().getX();
    l_next_target(1) = l_des_ee_pose.getOrigin().getY();
    l_next_target(2) = l_des_ee_pose.getOrigin().getZ();
    l_next_target(3) = l_des_ee_pose.getRotation().getX();
    l_next_target(4) = l_des_ee_pose.getRotation().getY();
    l_next_target(5) = l_des_ee_pose.getRotation().getZ();
    l_next_target(6) = l_des_ee_pose.getRotation().getW();

    l_cdd_cart_filter->SetTarget(l_next_target);
    l_cdd_cart_filter->Update();
    l_cdd_cart_filter->GetState(l_next_target);

    l_des_ee_pose.setOrigin(tf::Vector3(l_next_target(0), l_next_target(1), l_next_target(2)));
    l_des_ee_pose.setRotation(tf::Quaternion(l_next_target(3), l_next_target(4), l_next_target(5), l_next_target(6)));;

}

void BimanualActionServer::apply_task_specific_transformations(tf::Pose& left_final_target, tf::Pose& l_mNextRobotEEPose){

    // Transformations specific to the PEELING_TASK, the PHASE_REACH_TO_PEEL Models

    tf::Transform  l_ee_rot, ee_2_rob;
    l_ee_rot.setIdentity(); ee_2_rob.setIdentity();

    // Transform Attractor to Origin
    l_des_ee_pose.mult(left_final_target.inverse(),l_mNextRobotEEPose);

    // -> Apply Rotation (pi on Z in Origin RF)
    l_ee_rot.setBasis(tf::Matrix3x3(-1,0,0,0,-1,0,0,0,1)); //z (pi)
    l_des_ee_pose.mult(l_ee_rot,l_des_ee_pose);

    // -> Apply Rotation (pi on X in Origin RF)
    //l_ee_rot.setBasis(tf::Matrix3x3(1,0,0,0,cos,-sin,0,sin,cos)); //x
    l_ee_rot.setBasis(tf::Matrix3x3(1,0,0,0,0,1,0,-1,0)); //x (-pi/2)
    l_des_ee_pose.mult(l_ee_rot,l_des_ee_pose);

    // -> Transform back to Robot
    l_des_ee_pose.mult(left_final_target,l_des_ee_pose);
    l_des_ee_pose.setRotation(l_curr_ee_pose.getRotation().slerp(left_final_target.getRotation(), 0.75) );

    // Don't Care about master
    if (r_pos_err > 0.02)
        r_des_ee_pose = r_curr_ee_pose;

}


// Send desired EE_pose to robot/joint_ctrls.


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



// ======================================

bool BimanualActionServer::read_action_specification(TaskPhase phase, string model_base_path){

    char sActionFile[1025];
    sprintf(sActionFile, "%s/Phase%d/action_specification.txt", model_base_path.c_str(), phase);

    ROS_INFO_STREAM("Reading action specification from file: " << sActionFile);

    ifstream inputFile(sActionFile);

    string line;
    int line_number = 0;

    /*
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
*/

    while (getline(inputFile, line))
    {
        line_number++;
        switch(line_number){
        case 1:
            break;
        case 2:
            break;
        default:
            break;

        }
        istringstream ss(line);
        string name;
        int var;

        ss >> name >> var;

    }

    return true;
}


void BimanualActionServer::read_grasp_specification(TaskPhase phase, string model_base_path, std::string role, int arm_id){

    string arm;
    if (arm_id == R_ARM_ID)
        arm = "Right";
    if (arm_id == L_ARM_ID)
        arm = "Left";

    char sGraspFile[1025];
    sprintf(sGraspFile, "%s/Phase%d/%s/grasp_specification.txt", model_base_path.c_str(), phase, arm.c_str());

    ROS_INFO_STREAM("Reading grasp specification from file: " << sGraspFile);

    ROS_INFO_STREAM("Parameters: JA Mask >> JA avg >> JA deltas" );

    ifstream inputFile(sGraspFile);

    string line;
    int line_number; line_number = 0;

    while (getline(inputFile, line))
    {
        ROS_INFO_STREAM("Finger JA: " << line);
        istringstream ss(line);
        if (line_number < 22){
            ss >> finger_joints_mask(line_number) >> finger_joints_avg(line_number) >> finger_joints_deltas(line_number);
        }
        line_number++;

    }

    ROS_INFO_STREAM(" ******* Grasp Read Successfully ******* ");

}
