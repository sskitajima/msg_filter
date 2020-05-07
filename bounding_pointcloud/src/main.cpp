#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <sensor_msgs/PointCloud2.h>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

// #include <string>

using namespace std;
using message_filters::Subscriber;
using message_filters::TimeSynchronizer;



namespace bounding_pointcloud
{
    bool IsfirstImage = true;
    bool IsfirstPoint = true;
    bool IsfirstBbox = true;

    void callback1(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::PointCloud2ConstPtr &pc)
    {
        // 処理を書く
        ROS_INFO("callback1.");

    }

    void callback2(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::CameraInfoConstPtr &cam_info)
    {
        // Solve all of perception here...
        ROS_INFO("callback2.");

    }

    void callback3(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::PointCloud2ConstPtr &pc, const darknet_ros_msgs::BoundingBoxesConstPtr &bbox)
    {
        ROS_INFO("callback3.");

        // bboxの座標を取得

        // その範囲内にあるpointcloudを取得

        // それらの点からなるoctomapを構成

        // octomapをpublish
    }

    void debugCallback1(const sensor_msgs::ImageConstPtr &image)
    {
        if(IsfirstImage) ROS_INFO("first image.");
        IsfirstImage = false;
    }

    void debugCallback2(const sensor_msgs::PointCloud2ConstPtr &pc)
    {
        if(IsfirstPoint) ROS_INFO("first point.");
        IsfirstPoint = false;
    }

    void debugCallback3(const darknet_ros_msgs::BoundingBoxesConstPtr &bbox)
    {
        if(IsfirstBbox) ROS_INFO("first bbox.");
        IsfirstBbox = false;
    }

} // namespace bounding_pointcloud

int main(int argc, char** argv)
{
    const char* node_name = "img_filter_node";
    // const char* image_topic_name = "/camera/republish/rgb/image_color";
    const char* image_topic_name = "/darknet_ros_msgs/detection_image";
    const char* point_topic_name = "/camera/republish/rgb/points";
    const char* bbox_topic_name = "/darknet_ros/bounding_boxes";
    const char* camera_info_topic_name = "/camera/republish/rgb/image_info";

    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ROS_INFO("%s start.", node_name);


    Subscriber<sensor_msgs::Image> image_sub(nh, image_topic_name, 1);
    Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, point_topic_name, 1);
    TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2> sync1(image_sub, pc_sub, 100000);
    sync1.registerCallback(boost::bind(&bounding_pointcloud::callback1, _1, _2));

    // Subscriber<sensor_msgs::CameraInfo> info_sub(nh, camera_info_topic_name, 1);
    // TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> sync2(image_sub, info_sub, 10000000);
    // sync2.registerCallback(boost::bind(&bounding_pointcloud::callback2, _1, _2));

    // Subscriber<darknet_ros_msgs::BoundingBoxes> bbox_sub(nh, bbox_topic_name, 1);
    // TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2, darknet_ros_msgs::BoundingBoxes> sync3(image_sub, pc_sub, bbox_sub, 10000000);
    // sync3.registerCallback(boost::bind(&bounding_pointcloud::callback3, _1, _2, _3));



    ros::Subscriber sub_img = nh.subscribe(image_topic_name, 100, bounding_pointcloud::debugCallback1);
    ros::Subscriber sub_point = nh.subscribe(point_topic_name, 100, bounding_pointcloud::debugCallback2);
    ros::Subscriber sub_bbox = nh.subscribe(bbox_topic_name, 100, bounding_pointcloud::debugCallback3);
    
    
    ros::spin();

    return 0;
}