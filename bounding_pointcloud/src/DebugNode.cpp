#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/PointCloud2.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "cv_bridge/cv_bridge.h"

#include "bounding_pointcloud_msgs/CropImage.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <nlohmann/json.hpp>        // jsonを扱うライブラリ

#include <fstream>
#include <map>
#include <set>
#include <vector>

using namespace std;
namespace enc = sensor_msgs::image_encodings;

// トピック名をキーにして各トピックの受信回数を保持
std::map<const char *, uint32_t> topic_map;

void printHeader(const std_msgs::Header &header, const char *topic_name)
{
    topic_map[topic_name] += 1;
    cout << "topic: " << topic_name << " count: " << topic_map[topic_name] << endl;
    std::cout << "header:" << header.stamp << " seq: " << header.seq << std::endl;
    std::cout << std::endl;
}


template<class T>
class MySubscriber
{
private:
    // ros::NodeHandle nh_;
    ros::Subscriber sub_;
    const char* topic_name_;

public:
    MySubscriber(ros::NodeHandle& nh, const char* topic_name)
    : topic_name_(topic_name)
    {
        sub_ = nh.subscribe(topic_name_, 100, &MySubscriber::Callback, this);
        topic_map[topic_name_] = 0;
    }

    void Callback(const T& msg)
    {
        std::cout << "MySubscriber" << std::endl;
        printHeader(msg.header, topic_name_);
    }
};

class BoundingBoxSaver
{
private:
    vector<darknet_ros_msgs::BoundingBox> cur_detection, prev_detection; 
    vector<vector<darknet_ros_msgs::BoundingBox>> result;

    // multiset<int> cur_id;
    // set<multiset<int>> detection_ids;

public:
    BoundingBoxSaver()
    {
    }

    ~BoundingBoxSaver()
    {
        write_result();
        write_result_json();
    }

    void save_detections(const darknet_ros_msgs::BoundingBoxes& msg)
    {
        for(unsigned i=0; i < msg.bounding_boxes.size(); i++)
        {
            cur_detection.push_back(msg.bounding_boxes[i]);
            // cout << msg.bounding_boxes[i].id << endl;
            // cur_id.insert(msg.bounding_boxes[i].id);
        }

        // detection_ids.insert(cur_id);
        
        if(cur_detection != prev_detection) result.push_back(cur_detection);

        prev_detection = cur_detection;
        cur_detection.clear();

        cout << "num set detections: "  << result.size() << endl;
    }

    void write_result()
    {

        cout << "result size: " << result.size() << endl;
        string path = "/home/kitajima/workspace/save_image_detection/detection_result.txt";
        ofstream out_file;
        out_file.open(path);
        for(int i=0; i<result.size(); i++)
        {
            cout << "i " << i << endl;
            out_file << "image num: " << i << endl;;
            for(unsigned j=0; j<result[i].size(); j++)
            {
                darknet_ros_msgs::BoundingBox bbox = result[i][j];
                out_file << "Class: " << bbox.Class << " prob: " << bbox.probability << " id: " << bbox.id << '\n';
                out_file << "xmin: " << bbox.xmin << '\n';
                out_file << "ymin: " << bbox.ymin << '\n';
                out_file << "xmax: " << bbox.xmax << '\n';
                out_file << "ymax: " << bbox.ymax << '\n';
                out_file << endl;
            }
            out_file << endl;
        }
        out_file.close();
    }

    void write_result_json()
    {
        string path = "/home/kitajima/workspace/detection_result.json";

        for(int i=0; i<result.size(); i++)
        {
            string path = "/home/kitajima/workspace/save_image_detection/detection_result_" + to_string(i) + ".json";

            
            string result_str = "{ \"detections\" : ";
            string bbox_result_str = "[ ";
            for(unsigned j=0; j<result[i].size(); j++)
            {

                darknet_ros_msgs::BoundingBox bbox = result[i][j];
                string bbox_info_str = "{ \"Class\" : \""  + bbox.Class           + "\", "
                                    + "\"prob\" : " + to_string(bbox.probability) + ", "
                                    + "\"id\" : "   + to_string(bbox.id)          + ", "
                                    + "\"xmin\" : " + to_string(bbox.xmin)        + ", "
                                    + "\"ymin\" : " + to_string(bbox.ymin)        + ", "
                                    + "\"xmax\" : " + to_string(bbox.xmax)        + ", "
                                    + "\"ymax\" : " + to_string(bbox.ymax)        + "}"
                                    ;

                (j==0) ? bbox_result_str += bbox_info_str : bbox_result_str += ", " + bbox_info_str;                
                    
            }
            bbox_result_str += " ]";
            
            result_str += bbox_result_str + "}";
            nlohmann::json json_ = nlohmann::json::parse(result_str);
            std::ofstream o(path);
            o << std::setw(4) << json_ << std::endl;
            o.close();
        }
    }
};

class DebugNode
{
private:
    ros::NodeHandle nh;
    ros::Subscriber img_sub1;
    ros::Subscriber img_sub2;
    ros::Subscriber pc_sub;
    ros::Subscriber bbox_sub;
    ros::Subscriber info_sub;
    ros::Subscriber cropimg_sub;


    MySubscriber<bounding_pointcloud_msgs::CropImage> cropImg_sub;
    MySubscriber<sensor_msgs::PointCloud2> crop_points_sub;

    BoundingBoxSaver bboxSaver;

    const char* img_sub1_topic_;
    const char* bbox_sub_topic_;
    const char* pc_sub_topic_;
    const char* info_sub_topic_;
    const char* croping_sub_topic_;


public:
    DebugNode(const char* img_sub1_topic, const char* bbox_sub_topic, const char* pc_sub_topic, const char* info_sub_topic, const char* croping_sub_topic)
    : img_sub1_topic_(img_sub1_topic), 
      bbox_sub_topic_(bbox_sub_topic), 
      pc_sub_topic_(pc_sub_topic), 
      info_sub_topic_(info_sub_topic),
      croping_sub_topic_(croping_sub_topic),
      cropImg_sub(nh, croping_sub_topic),
      crop_points_sub(nh, "/points_croped/points")
    {
        // img_sub1 = nh.subscribe(img_sub1_topic_, 100, &DebugNode::img_sub1_callback, this);
        bbox_sub = nh.subscribe(bbox_sub_topic_, 100, &DebugNode::bbox_sub_callback, this);
        // pc_sub = nh.subscribe(pc_sub_topic_, 100, &DebugNode::pc_sub_callback, this);
        // info_sub = nh.subscribe(info_sub_topic, 100, &DebugNode::info_sub_callback, this);
        // cropimg_sub = nh.subscribe(cropimg_sub_topic_, 100, &DebugNode::cropimg_sub_callback, this);

        topic_map[img_sub1_topic_] = 0;
        topic_map[bbox_sub_topic_] = 0;
        topic_map[pc_sub_topic_] = 0;
        topic_map[info_sub_topic_] = 0;
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
        bboxSaver.save_detections(msg);
    }

    void pc_sub_callback(const sensor_msgs::PointCloud2& msg)
    {
        printHeader(msg.header, pc_sub_topic_);
    }

    void info_sub_callback(const sensor_msgs::CameraInfo& msg)
    {
        printHeader(msg.header, info_sub_topic_);
    }

    void cropimg_sub_callback(const bounding_pointcloud_msgs::CropImage& img_msg)
    {
        cout << "===CropImage subscribe===" << endl;
        // cv_bridge::CvImagePtr cv_rgb_ptr, cv_depth_ptr;
        // try
        // {
        //     cv_rgb_ptr = cv_bridge::toCvCopy(img_msg.rgb_image, enc::BGR8);
        //     cv_depth_ptr = cv_bridge::toCvCopy(img_msg.depth_image, enc::TYPE_32FC1);
        // }
        // catch (cv_bridge::Exception &ex)
        // {
        //     ROS_ERROR("error");
        //     exit(-1);
        // }
        
        // cv::Mat dst;
        // cv::normalize(cv_depth_ptr->image, dst, 0, 1, cv::NORM_MINMAX);
        // cv::imshow("rgb image", cv_rgb_ptr->image);
        // cv::imshow("depth image", dst);
        // cv::waitKey(2);

        // cout << endl;
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "debug_node");
    // if(argc != 4){
    //     cout << "usgage: rosrun bounding_pointcloud debug_node [subsctibe_topic_name] [bbox_topic_name] [pointcloud_topic_name]" << endl;
    // }
    const char* img_topic_name = "/camera/rgb/image_color";
    const char* bbox_topic_name = "/darknet_ros/bounding_boxes";
    const char* pc_topic_name = "/camera/republish/rgb/points";
    const char* info_topic_name = "/camera/rgb/camera_info";
    const char* croping_topic_name = "bounding_pointcloud_msgs/CropImage";

    // vector<const char*> topic_names{img_topic_name,
    //                                 bbox_topic_name,
    //                                 pc_topic_name,
    //                                 info_topic_name
    //                                 }

    DebugNode node(img_topic_name, bbox_topic_name, pc_topic_name, info_topic_name, croping_topic_name);

    ros::spin();

}