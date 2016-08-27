#include "bimanual_action_server.h"

bool BimanualActionServer::coordinated_bimanual_ds_execution(TaskPhase phase, tf::Transform task_frame, tf::Transform right_att, tf::Transform left_att, double dt){

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
    vo_dsRun->init(dt,1.0,0.5,800.0,400.0,400.0);
    vo_dsRun->setCurrentObjectState(real_object, real_object_velocity);
    vo_dsRun->setInterceptPositions(real_object, left_final_target, right_final_target);
    vo_dsRun->setCurrentEEStates(l_curr_ee_pose,r_curr_ee_pose);
    vo_dsRun->initializeVirtualObject();

    // Vairable for execution
    ros::Duration loop_rate(dt);
    static tf::TransformBroadcaster br;
    tf::Transform r_trans_ee, l_trans_ee;
    double object_err;


    // Before Starting a Reach Bias the FT-Sensors!
    biasFtSensors();


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

        // Current progress variable (position)
        object_err = (virtual_object.getOrigin() - real_object.getOrigin()).length();
        reachingThreshold = 0.017;
        ROS_INFO_STREAM("Position Threshold : "    << reachingThreshold    << " ... Current VO Error: " << object_err);

        as_.publishFeedback(feedback_);

        // // Only Check for Position Error
        if(object_err < reachingThreshold ) {

            ROS_INFO_STREAM("VIRTUAL OBJECT HAS CONVERGED!!");
//            sendPose(r_curr_ee_pose, l_curr_ee_pose);

            if(phase ==  PHASE_INIT_REACH) {
                ROS_INFO_STREAM("In PHASE_INIT_REACH.. finding table now...");
                if (bWaitForForces_right_arm)	{
                    bool x_r_arm = find_object_by_contact(R_ARM_ID, 0.07, 0.05, 7);
                    return x_r_arm;
                }
            }else if(phase ==  PHASE_RETRACT){
                ROS_INFO_STREAM("In PHASE_INIT_RETRACT.. biasing ft sensors...");
                biasFtSensors();
            }
            break;
        }

        loop_rate.sleep();
    }
    delete vo_dsRun;

    return ros::ok();

}


void BimanualActionServer::compute_object_pose(const tf::Transform& right, const tf::Transform& left, tf::Transform& object){

    // Compute object position (midpoint between left and right reaching points)
    tf::Vector3 o_pos = left.getOrigin() + (right.getOrigin() - left.getOrigin())/2;
    tf::Vector3 tmp(right.getOrigin());
    //    tmp.setX(tmp.getX() + 0.05);
    tmp.setX(tmp.getX() + 0.05);

    // Compute object orientation (xdir: direction from right to left reaching point, ydir: y plane in world rf)
    Eigen::Vector3d xdir (right.getOrigin().getX() - left.getOrigin().getX(), right.getOrigin().getY() - left.getOrigin().getY(),
                          right.getOrigin().getZ() - left.getOrigin().getZ());

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

// Publish real object shape for VO bimanual DS
void BimanualActionServer::publish_ro_rviz(const tf::Transform& ro_pose, const double& object_length){

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
void BimanualActionServer::publish_vo_rviz(const tf::Transform& vo_pose,  const double& object_length,
                                           const tf::Transform& right, const tf::Transform& left)
{

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
