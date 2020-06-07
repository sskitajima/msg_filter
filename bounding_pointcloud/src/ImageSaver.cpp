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
    const std::string save_image_dir_d = "/home/kitajima/workspace/save_image_depth/";
    const float SENSOR_MAX_DEPTH = 3.5;
    int counter = 0;

public:
    ImageSaver()
    : it_img(nh_),
      it_depth_filter(nh_),
      it_img_filter(nh_)
    {
        int queue_size = 10000;
        // it_ptr.reset(new image_transport::ImageTransport(nh));
        //sub_img = it_img.subscribe(topic_save_image, 1, &ImageSaver::imageCallback, this);

        image_transport::TransportHints img_hints("raw", ros::TransportHints(), nh_, "img_hints");
        image_transport::TransportHints depth_hints("raw", ros::TransportHints(), nh_, "depth_hints");
        sub_img_filter.subscribe(it_img_filter, topic_save_image, 1, img_hints);
        sub_depth_filter.subscribe(it_depth_filter, topic_save_depth, 1, depth_hints);

        sync_.reset(new Synchronizer(SyncPolicy(queue_size), sub_img_filter, sub_depth_filter));
        sync_->registerCallback(boost::bind(&ImageSaver::imageFilterCallback, this, _1, _2));

        ROS_INFO("ImageSaver Constructor Finished.");
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

    // 画像をopencvとrosで変換するための変数
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img_msg, enc::BGR8);
    }
    catch (cv_bridge::Exception &ex)
    {
        ROS_ERROR("rgb encoding error: %s", ex.what());
        exit(-1);
    }

    cv_bridge::CvImagePtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvCopy(depth_msg, enc::TYPE_32FC1);
    }
    catch (cv_bridge::Exception &ex)
    {
        ROS_ERROR("depth encoding error: %e", ex.what());
        exit(-1);
    }


    // save image
    const std::string save_path = (boost::format("%skeyframe_%d.png") % save_image_dir % counter).str();
    imwrite(save_path, cv_ptr->image);

    const std::string save_path_d = (boost::format("%skeyframe_%d.png") % save_image_dir_d % counter).str();


    // cv::Mat depth(cv_ptrD->image.rows, cv_ptrD->image.cols, CV_32FC1);
    // cv::Mat depth_img(cv_ptrD->image.rows, cv_ptrD->image.cols, CV_8UC1);

    cv::Mat depth;
    (cv_ptrD->image).convertTo(depth, CV_8UC1, 255/SENSOR_MAX_DEPTH);
    // cv::normalize(cv_ptrD->image, depth, cv::NormTypes::NORM_MINMAX);
    // (depth).convertTo(depth2, CV_8UC1, 255);

    // 結果画像表示
    cv::namedWindow("Image convert", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
    cv::imshow("Image convert", depth);
    cv::waitKey(1);

    imwrite(save_path_d, depth);
    counter++;
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
