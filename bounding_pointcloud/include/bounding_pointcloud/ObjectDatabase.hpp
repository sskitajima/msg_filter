#ifndef OBJECT_DATABASE_HPP_
#define OBJECT_DATABASE_HPP_

#include "ros/ros.h"
#include <nodelet/nodelet.h>


namespace bounding_pointcloud
{

class ObjectDatabase : public nodelet::Nodelet
{
private:
    virtual void onInit();

public:
    ros::NodeHandle nh_;
};

}  // namespace bounding_pointcloud
#endif