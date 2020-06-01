#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <string>
#include <boost/format.hpp>

using namespace std;
namespace enc = sensor_msgs::image_encodings;

namespace bounding_pointcloud
{

class ImageSaver
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_img;

    const char* topic_save_image = "/orb_slam2_rgbd/keyframe";
    const std::string save_image_dir = "/home/kitajima/workspace/save_image/";
    int counter = 0;

public:
    ImageSaver()
    : it_(nh_)
    {
        // it_ptr.reset(new image_transport::ImageTransport(nh));
        sub_img = it_.subscribe(topic_save_image, 1, &ImageSaver::imageCallback, this);
    }

    ~ImageSaver()
    {
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& img_msg);
};

void ImageSaver::imageCallback(const sensor_msgs::ImageConstPtr &img_msg)
{
    ROS_INFO("imageCallback");

    // 画像をopencvとrosで変換するための変数
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        // cout << "encoding; " << img_msg->encoding << endl;
        if (enc::isColor(img_msg->encoding))
            cv_ptr = cv_bridge::toCvCopy(img_msg, enc::BGR8);
        else
            cv_ptr = cv_bridge::toCvCopy(img_msg, enc::TYPE_32FC1);
    }
    catch (cv_bridge::Exception &ex)
    {
        ROS_ERROR("encoding error");
        exit(-1);
    }

    // save image
    const std::string save_path = (boost::format("%skeyframe_%d.png") % save_image_dir % counter).str();
    imwrite(save_path, cv_ptr->image);
    counter++;
}

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imageSaver_node");
    bounding_pointcloud::ImageSaver imageSaver;
    ros::spin();
}
