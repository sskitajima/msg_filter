#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>       
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <boost/timer.hpp>

#include <darknet_ros_msgs/BoundingBoxes.h>

namespace bounding_pointcloud {

using namespace message_filters::sync_policies;
namespace enc = sensor_msgs::image_encodings;

class ImageCropExample : public nodelet::Nodelet
{
  ros::NodeHandlePtr rgb_nh_, depth_nh_;
  ros::NodeHandlePtr bbox_nh_;
  boost::shared_ptr<image_transport::ImageTransport> rgb_it_, depth_it_;
  
  // Subscriptions
  image_transport::SubscriberFilter sub_rgb_, sub_depth_;
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> sub_bbox_;

  typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  boost::shared_ptr<Synchronizer> sync_;

  // Publications
  ros::Publisher pub_crop_img;

  virtual void onInit();

  void crop(const sensor_msgs::ImageConstPtr& rgb_msg,
            const sensor_msgs::ImageConstPtr& depth_msg,
            const darknet_ros_msgs::BoundingBoxesConstPtr& bbox);
};

void ImageCropExample::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  ros::NodeHandle depth_nh(nh, "depth_registered");

  rgb_nh_  .reset( new ros::NodeHandle(nh, "camera/rgb") );
  depth_nh_.reset( new ros::NodeHandle(nh, "camera/depth") );
  bbox_nh_ .reset( new ros::NodeHandle(nh) );

  rgb_it_  .reset( new image_transport::ImageTransport(*rgb_nh_) );
  depth_it_.reset( new image_transport::ImageTransport(*depth_nh_) );
  // depth_it_.reset( new image_transport::ImageTransport(depth_nh) );
  
  // Read parameters
  int queue_size;
  //　TODO: 調整する必要あり
  nh.param("queue_size", queue_size, 10);

  sync_.reset(new Synchronizer(SyncPolicy(queue_size), sub_rgb_, sub_depth_, sub_bbox_));
  sync_->registerCallback(boost::bind(&ImageCropExample::crop, this, _1, _2, _3));

  // parameter for depth_image_transport hint
  // std::string depth_image_transport_param = "depth_image_transport";

  image_transport::TransportHints hints_rgb("raw", ros::TransportHints(), private_nh);
  image_transport::TransportHints hints_depth("raw", ros::TransportHints(), private_nh/*, depth_image_transport_param*/);
  sub_rgb_  .subscribe(*rgb_it_  , "image_color", 1, hints_rgb);
  sub_depth_.subscribe(*depth_it_, "image"      , 1, hints_depth);
  sub_bbox_ .subscribe(*bbox_nh_ , "darknet_ros/bounding_boxes", 1);

  pub_crop_img = nh.advertise<sensor_msgs::Image>("/camera/rgb/image_color_croped", 100);
  NODELET_INFO("ImageCropExample: initialize finished.");
}

void ImageCropExample::crop(const sensor_msgs::ImageConstPtr& rgb_msg,
                            const sensor_msgs::ImageConstPtr& depth_msg,
                            const darknet_ros_msgs::BoundingBoxesConstPtr& bbox_msg)
{
    NODELET_INFO("crop start.");
    for (int bbox_id = 0; bbox_id < bbox_msg->bounding_boxes.size(); bbox_id++)
    {
      // boost::timer t;

      // クロップ処理を行う
      cv_bridge::CvImagePtr cv_ptr;

      NODELET_INFO("encoding_type:");
      if (rgb_msg->encoding == enc::BGR8)
      {
        std::cout << rgb_msg->encoding << std::endl;

        try
        {
          cv_ptr = cv_bridge::toCvCopy(rgb_msg, enc::BGR8);
          std::cout << "try success" << std::endl;
        }
        catch (cv_bridge::Exception &ex)
        {
          ROS_ERROR("error");
          exit(-1);
        }

        cv::Mat &mat = cv_ptr->image;

        darknet_ros_msgs::BoundingBox bbox = bbox_msg->bounding_boxes[bbox_id];

        for (int i = bbox.ymin; i < bbox.ymax; i++)
        {
          for (int j = bbox.xmin; j < bbox.xmax; j++)
          {
            // 0 : blue, 1 : green, 2 : red.
            mat.data[i * mat.step + j * mat.elemSize() + 0] = 0;
            mat.data[i * mat.step + j * mat.elemSize() + 1] = 0;
            //mat.data[i * mat.step + j * mat.elemSize() + 2] = 0;
          }
        }

        cv::rectangle(cv_ptr->image, cv::Point(bbox.xmin, bbox.ymin), cv::Point(bbox.xmax, bbox.ymax), cv::Scalar(0, 0, 255), 3, 4);
        // cv::imshow("RGB image", cv_ptr->image);
        pub_crop_img.publish(cv_ptr->toImageMsg());
        // NODELET_INFO("time for making and publishing one image: %d", t.elapsed());
      }

    }

    NODELET_INFO("crop end");
    std::cout << std::endl;
}


} // namespace bounding_pointcloud

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(bounding_pointcloud::ImageCropExample,nodelet::Nodelet);
