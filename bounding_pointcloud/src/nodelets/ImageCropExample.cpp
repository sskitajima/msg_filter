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
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <darknet_ros_msgs/BoundingBoxes.h>

namespace bounding_pointcloud {

using namespace message_filters::sync_policies;

class ImageCropExample : public nodelet::Nodelet
{
  ros::NodeHandlePtr rgb_nh_;
  ros::NodeHandlePtr bbox_nh_;
  boost::shared_ptr<image_transport::ImageTransport> rgb_it_;
  
  // Subscriptions
  image_transport::SubscriberFilter sub_rgb_;
  // message_filters::Subscriber<sensor_msgs::Image> sub_rgb_;
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> sub_bbox_;

  typedef ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  boost::shared_ptr<Synchronizer> sync_;

  // Publications
  ros::Publisher pub_crop_img;

  virtual void onInit();

  void crop(const sensor_msgs::ImageConstPtr& rgb_msg,
            const darknet_ros_msgs::BoundingBoxesConstPtr& bbox);
};

void ImageCropExample::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();

  rgb_nh_ .reset( new ros::NodeHandle(nh, "rgb") );
  bbox_nh_.reset( new ros::NodeHandle(nh) );

  rgb_it_.reset( new image_transport::ImageTransport(*rgb_nh_) );
  image_transport::TransportHints hints("raw", ros::TransportHints(), private_nh);
  sub_rgb_ .subscribe(*rgb_it_, "image_color", 1, hints);
  sub_bbox_.subscribe(*bbox_nh_, "darknet_ros/bounding_boxes", 1);

  // Read parameters
  int queue_size;
  nh.param("queue_size", queue_size, 100000);

  sync_.reset(new Synchronizer(SyncPolicy(queue_size), sub_rgb_, sub_bbox_));
  sync_->registerCallback(boost::bind(&ImageCropExample::crop, this, _1, _2));

  pub_crop_img = nh.advertise<sensor_msgs::Image>("/camera/rgb/croped/image_color", 100);
  NODELET_INFO("initialize finished. example");
}

void ImageCropExample::crop(const sensor_msgs::ImageConstPtr& rgb_msg,
                            const darknet_ros_msgs::BoundingBoxesConstPtr& bbox)
{
    NODELET_INFO("crop start.");

    // クロップ処理を行う

    // pub_crop_img.publish(crop_img);

    NODELET_INFO("crop end");
    std::cout << std::endl;
}


} // namespace bounding_pointcloud

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(bounding_pointcloud::ImageCropExample,nodelet::Nodelet);
