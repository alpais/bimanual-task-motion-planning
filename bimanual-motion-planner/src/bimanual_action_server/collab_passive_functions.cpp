#include "bimanual_action_server.h"

bool BimanualActionServer::collab_passive_model_execution(TaskPhase phase, tf::Transform task_frame, tf::Transform right_att, double dt){


    ROS_INFO_STREAM(" Model Path "                  << model_base_path);
    ROS_INFO_STREAM(" Execute Learned model: phase "<< phase);
    ROS_INFO_STREAM(" Reaching threshold "          << reachingThreshold);
    ROS_INFO_STREAM(" Orientation threshold "       << orientationThreshold);
    ROS_INFO_STREAM(" Model DT "                    << model_dt);

    tf::Transform  right_final_target;


}


