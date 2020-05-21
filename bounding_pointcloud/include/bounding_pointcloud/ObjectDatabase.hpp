#ifndef OBJECT_DATABASE_HPP_
#define OBJECT_DATABASE_HPP_

#include "ros/ros.h"
#include <nodelet/nodelet.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <string>



namespace bounding_pointcloud
{

class DetectionData
{
private:
    std_msgs::Header header_;
    std::string class_;
    cv::Mat& image_;
    boost::shared_ptr<sensor_msgs::CameraInfo const> cam_info_ptr_;

public:
    DetectionData(
            std_msgs::Header header, 
            const std::string& Class, 
            cv::Mat& image,
            boost::shared_ptr<sensor_msgs::CameraInfo const> cam_info_ptr
            ) 
       : header_(header),
         class_(Class),
         image_(image),
         cam_info_ptr_(cam_info_ptr)

    {
        std::cout << "register DetectionData" << std::endl;
        print();
    }

    ~DetectionData()
    {

    }

    void print()
    {
        std::cout << std::endl;
        std::cout << "header.stamp: " << header_.stamp << std::endl;
        std::cout << "class: " << class_ << std::endl;
        // cv::imshow(class_+" image", image_);
        std::cout << std::endl;
    }

};

class RegisterObject : public nodelet::Nodelet
{
private:
    virtual void onInit();


public:
    ros::NodeHandle nh_;
    image_transport::SubscriberFilter sub_rgb_, sub_depth_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
};

}  // namespace bounding_pointcloud
#endif