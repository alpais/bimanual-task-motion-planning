#include <ros/ros.h>
#include "actionlib/server/simple_action_server.h"
#include "bimanual_action_server.h"

int 
main (int argc, char **argv)
{
    ros::init(argc, argv, "bimanual_plan2ctrl");

    ROS_INFO("Initializing Server");
    BimanualActionServer action_execution(ros::this_node::getName());
    action_execution.initialize();

    ros::spin();
    return 0;
}
