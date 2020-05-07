#include "ros/ros.h"
#include "bounding_pointcloud/SynchnornizerNode.hpp"

int main(int argc, char** argv)
{
    const char* node_name = "synchronizer_node";
    ros::init(argc, argv, node_name);
    SynchronizerNode node;

    ros::spin();
}