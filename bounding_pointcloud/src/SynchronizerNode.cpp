#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "bounding_pointcloud/SynchnornizerNode.hpp"

using namespace std;

template<class T>
bool check_contain_value_nd(const std::vector<T*> points, const std::vector<T> min_values, const std::vector<T> max_values)
{
    assert(points.size() == min_values.size() == max_values.size());
    
    // 各次元について計算
    for (int dim = 0; dim < points.size(); dim++)
    {
        if(*(points[dim]) >= min_values[dim] && *(points[dim]) <= max_values[dim])continue;
        else return false;
    }

    return true;
}

SynchronizerNode::SynchronizerNode()
{
    sub_img.subscribe(nh, image_topic_name, 10);
    sub_pc.subscribe(nh, point_topic_name, 10);
    sub_bbox.subscribe(nh, bbox_topic_name, 10);

    sub_scan1.subscribe(nh, test_topic_name1, 10);
    sub_scan2.subscribe(nh, test_topic_name2, 10);

    sync3.reset(new Sync3(MySyncPolicy3(1000), sub_img, sub_pc, sub_bbox));
    sync3->registerCallback(boost::bind(&SynchronizerNode::callback3, this, _1, _2, _3));

    sync1.reset(new Sync1(MySyncPolicy1(1000), sub_img, sub_pc));
    sync1->registerCallback(boost::bind(&SynchronizerNode::callback1, this, _1, _2));

    sync_test.reset(new SyncTest(MySyncPolicyTest(1000), sub_scan1, sub_scan2));
    sync_test->registerCallback(boost::bind(&SynchronizerNode::callback_test, this, _1, _2));

    pub_detected_pc = nh.advertise<sensor_msgs::PointCloud2>(out_pc_name, 100);

    ROS_INFO("synchronizer_node start.");
}

void SynchronizerNode::callback1(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::PointCloud2ConstPtr &pc)
{
    ROS_INFO("callback1.");
}

void SynchronizerNode::callback2(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::CameraInfoConstPtr &cam_info)
{
    ROS_INFO("callback2.");
}

void SynchronizerNode::callback3(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::PointCloud2ConstPtr &pc, const darknet_ros_msgs::BoundingBoxesConstPtr &bbox)
{
    ROS_INFO("callback3.");

    // bboxの座標を取得
    int num_bbox = bbox->bounding_boxes.size();
    for (int i = 0; i < num_bbox; i++)
    {
        // その範囲内にあるpointcloudを取得
        print_bbox(bbox->bounding_boxes[i]);

        // それらの点からなるoctomapを構成
        // 確認用にxyz座標のsensor_msgs::PointCloudに変換
        sensor_msgs::PointCloud pc_, result_pc_;
        sensor_msgs::convertPointCloud2ToPointCloud(*pc, pc_);

        std::vector<float> min_values = {bbox->bounding_boxes[i].xmin, bbox->bounding_boxes[i].ymin};
        std::vector<float> max_values = {bbox->bounding_boxes[i].xmax, bbox->bounding_boxes[i].ymax};
        std::vector<float*> query(2);

        std::vector<float> point_x, point_y, point_z;
        for(int j=0; j<pc_.points.size(); j++)
        {
            query[0] = &(pc_.points[j].x);
            query[1] = &(pc_.points[j].y);
            cout << "query: " << *query[0] << " " << *query[1] << endl;
            if(check_contain_value_nd(query, min_values, max_values))
            { 
                cout << "points: " << pc_.points[j].x << " " << pc_.points[j].y << endl;  
                result_pc_.points.push_back(pc_.points[j]);
                result_pc_.channels.push_back(pc_.channels[j]);
            }
        }

        sensor_msgs::PointCloud2 out_pc;
        sensor_msgs::convertPointCloudToPointCloud2(result_pc_, out_pc);
        out_pc.header.stamp = ros::Time::now();
        out_pc.header.frame_id = pc->header.frame_id;
        out_pc.header.seq = out_seq;
        out_seq++;

        // octomapをpublish
        pub_detected_pc.publish(out_pc);
        cout << endl;
    }
}

void SynchronizerNode::callback_test(const sensor_msgs::LaserScanConstPtr &scan1, const sensor_msgs::LaserScanConstPtr &scan2)
{
    ROS_INFO("callback test");

    std::cout << "scan1 header:" << scan1->header.stamp << " seq: " << scan1->header.seq << std::endl;
    std::cout << "scan2 header:" << scan2->header.stamp << " seq: " << scan2->header.seq << std::endl;
    std::cout << "diff: " << scan1->header.stamp - scan2->header.stamp << std::endl;
    std::cout << std::endl;
}

void SynchronizerNode::print_bbox(const darknet_ros_msgs::BoundingBox &bbox)
{
    std::cout << "Class: " << bbox.Class << std::endl;
    std::cout << "Prob: " << bbox.probability << std::endl;
    std::cout << "id: " << bbox.id << std::endl;
    std::cout << "corrd: (" << bbox.xmin << ", " << bbox.ymin << "), (" << bbox.xmax << ", " << bbox.ymax << ")" << std::endl;
}