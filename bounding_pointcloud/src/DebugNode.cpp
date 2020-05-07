#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "darknet_ros_msgs/BoundingBoxes.h"

#include <map>

using namespace std;

class DebugNode
{
private:
    ros::NodeHandle nh;
    ros::Subscriber img_sub1;
    ros::Subscriber img_sub2;
    ros::Subscriber pc_sub;
    ros::Subscriber bbox_sub;
    // ros::Subscriber img_sub;
    // ros::Subscriber img_sub;

    const char* img_sub1_topic_;
    const char* bbox_sub_topic_;
    const char* pc_sub_topic_;

    // トピック名をキーにして各トピックの受信回数を保持
    std::map<const char*, uint32_t> topic_map;


public:
    DebugNode(const char* img_sub1_topic, const char* bbox_sub_topic, const char* pc_sub_topic)
    : img_sub1_topic_(img_sub1_topic), bbox_sub_topic_(bbox_sub_topic), pc_sub_topic_(pc_sub_topic)
    {
        img_sub1 = nh.subscribe(img_sub1_topic_, 100, &DebugNode::img_sub1_callback, this);
        bbox_sub = nh.subscribe(bbox_sub_topic_, 100, &DebugNode::bbox_sub_callback, this);
        pc_sub = nh.subscribe(pc_sub_topic_, 100, &DebugNode::pc_sub_callback, this);

        topic_map[img_sub1_topic_] = 0;
        topic_map[bbox_sub_topic_] = 0;
        topic_map[pc_sub_topic_] = 0;
    }

    ~DebugNode()
    {

    }

    void img_sub1_callback(const sensor_msgs::Image& msg)
    {
        printHeader(msg.header, img_sub1_topic_);
    }

    void bbox_sub_callback(const darknet_ros_msgs::BoundingBoxes& msg)
    {
        printHeader(msg.header, bbox_sub_topic_);
    }

    void pc_sub_callback(const sensor_msgs::PointCloud2& msg)
    {
        printHeader(msg.header, pc_sub_topic_);
    }

    void printHeader(const std_msgs::Header &header, const char* topic_name)
    {
        topic_map[topic_name] += 1;
        cout << "topic: " << img_sub1_topic_ << " count: " << topic_map[topic_name] << endl;
        std::cout << "header:" << header.stamp <<  " seq: " << header.seq << std::endl;
        std::cout << std::endl;
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "debug_node");
    // if(argc != 4){
    //     cout << "usgage: rosrun bounding_pointcloud debug_node [subsctibe_topic_name] [bbox_topic_name] [pointcloud_topic_name]" << endl;
    // }
    const char* img_topic_name = "/darknet_ros/detection_image";
    const char* bbox_topic_name = "/darknet_ros/bounding_boxes";
    const char* pc_topic_name = "/camera/republish/rgb/points";

    DebugNode node(img_topic_name, bbox_topic_name, pc_topic_name);

    ros::spin();

}