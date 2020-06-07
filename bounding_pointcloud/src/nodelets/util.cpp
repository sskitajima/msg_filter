#include <ros/ros.h>
#include "std_msgs/Header.h"


std_msgs::Header createHeader(const std::string &frame_id_)
{
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = frame_id_;

  return header;
}