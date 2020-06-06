#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <string>
#include <boost/format.hpp>

using namespace std;
using namespace message_filters::sync_policies;
namespace enc = sensor_msgs::image_encodings;

namespace bounding_pointcloud
{

class ImageSaver
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_img, it_depth_filter, it_img_filter;
    image_transport::Subscriber sub_img;
    image_transport::SubscriberFilter sub_depth_filter, sub_img_filter;


    typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    boost::shared_ptr<Synchronizer> sync_;


    const char* topic_save_image = "/orb_slam2_rgbd/keyframe_rgb";
    const char* topic_save_depth = "/orb_slam2_rgbd/keyframe_depth";
    const std::string save_image_dir = "/home/kitajima/workspace/save_image/";
    int counter = 0;

public:
    ImageSaver()
    : it_img(nh_),
      it_depth_filter(nh_),
      it_img_filter(nh_)
    {
        int queue_size = 10000;
        // it_ptr.reset(new image_transport::ImageTransport(nh));
        sub_img = it_img.subscribe(topic_save_image, 1, &ImageSaver::imageCallback, this);

        image_transport::TransportHints img_hints("raw", ros::TransportHints(), nh_, "img_hints");
        image_transport::TransportHints depth_hints("raw", ros::TransportHints(), nh_, "depth_hints");
        sub_img_filter.subscribe(it_img_filter, topic_save_image, 1, img_hints);
        sub_depth_filter.subscribe(it_depth_filter, topic_save_depth, 1, depth_hints);

        sync_.reset(new Synchronizer(SyncPolicy(queue_size), sub_depth_filter, sub_img_filter));
        sync_->registerCallback(boost::bind(&ImageSaver::imageFilterCallback, this, _1, _2));
    }

    ~ImageSaver()
    {
    }

    void imageFilterCallback(const sensor_msgs::ImageConstPtr&, const sensor_msgs::ImageConstPtr&);

    void imageCallback(const sensor_msgs::ImageConstPtr& img_msg);
};

void ImageSaver::imageFilterCallback(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::ImageConstPtr& depth_msg)
{
    ROS_INFO("imageFilterCallback");
}

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
