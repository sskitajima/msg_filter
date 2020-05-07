#ifndef SYNCHRONIZER_NODE_H
#define SYNCHRONIZER_NODE_H

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

#include <limits>
#include <vector>
#include <string>
#include <cassert>

#include "pcl_ros/point_cloud.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;



/// 参考
/// https://answers.ros.org/question/280856/synchronizer-with-approximate-time-policy-in-a-class-c/
class SynchronizerNode
{
private:
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> sub_img;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pc;
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> sub_bbox;
    
    message_filters::Subscriber<sensor_msgs::LaserScan> sub_scan1;
    message_filters::Subscriber<sensor_msgs::LaserScan> sub_scan2;

    ros::Publisher pub_detected_pc;
    uint32_t out_seq = 0;


    const char* image_topic_name = "/camera/rgb/image_color";
    const char* point_topic_name = "/camera/rgb/points";
    const char* bbox_topic_name = "/darknet_ros/bounding_boxes";
    const char* camera_info_topic_name = "/camera/rgb/image_info";
    
    const char* test_topic_name1 = "/scan1";
    const char* test_topic_name2 = "/scan2";

    const char* out_pc_name = "/detection_pointcloud";


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2, darknet_ros_msgs::BoundingBoxes> MySyncPolicy3;
    typedef message_filters::Synchronizer<MySyncPolicy3> Sync3;
    boost::shared_ptr<Sync3> sync3;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy1;
    typedef message_filters::Synchronizer<MySyncPolicy1> Sync1;
    boost::shared_ptr<Sync1> sync1;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> MySyncPolicyTest;
    typedef message_filters::Synchronizer<MySyncPolicyTest> SyncTest;
    boost::shared_ptr<SyncTest> sync_test;

    ros::Subscriber sub_img_deb;
    ros::Subscriber sub_point_deb; 
    ros::Subscriber sub_bbox_deb;

public:
    SynchronizerNode();

    void callback1(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::PointCloud2ConstPtr &pc);
    void callback2(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::CameraInfoConstPtr &cam_info);
    void callback3(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::PointCloud2ConstPtr &pc, const darknet_ros_msgs::BoundingBoxesConstPtr &bbox);
    void callback_test(const sensor_msgs::LaserScanConstPtr &scan1, const sensor_msgs::LaserScanConstPtr &scan2);

    void print_bbox(const darknet_ros_msgs::BoundingBox &bbox);
};

#endif  // SYNCHRONIZER_NODE_H