#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <ros/topic.h>
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
#include <boost/format.hpp>

#include <darknet_ros_msgs/BoundingBoxes.h>

#include "bounding_pointcloud/ObjectDatabase.hpp"



namespace bounding_pointcloud {

using namespace message_filters::sync_policies;
namespace enc = sensor_msgs::image_encodings;

class ImageCrop : public nodelet::Nodelet
{
  // nodehandler
  ros::NodeHandlePtr rgb_nh_, depth_nh_;
  ros::NodeHandlePtr bbox_nh_;
  boost::shared_ptr<image_transport::ImageTransport> rgb_it_, depth_it_;
  boost::shared_ptr<sensor_msgs::CameraInfo const> cam_info_ptr_;
  
  // Subscriptions
  image_transport::SubscriberFilter sub_rgb_, sub_depth_;
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> sub_bbox_;

  // message sychtonizer
  typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  boost::shared_ptr<Synchronizer> sync_;

  // Publications
  ros::Publisher pub_crop_img;

  // Object Detection Database
  std::vector<bounding_pointcloud::DetectionData*> database;

  // topic name
  const std::string cam_topic_ = "/camera/rgb/camera_info";

  // save image param
  int counter=0;
  const std::string save_class_name = "book";
  const std::string save_image_dir = "/home/kitajima/workspace/save_image/";


  virtual void onInit();
  void crop(const sensor_msgs::ImageConstPtr& rgb_msg,
            const sensor_msgs::ImageConstPtr& depth_msg,
            const darknet_ros_msgs::BoundingBoxesConstPtr& bbox);
  void registerObject(const sensor_msgs::ImageConstPtr& rgb_msg,
                      const darknet_ros_msgs::BoundingBox& bbox,
                      const cv_bridge::CvImagePtr& cv_ptr);
  void count_object();
    
};

void ImageCrop::onInit()
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
  sync_->registerCallback(boost::bind(&ImageCrop::crop, this, _1, _2, _3));

  // parameter for depth_image_transport hint
  // std::string depth_image_transport_param = "depth_image_transport";

  image_transport::TransportHints hints_rgb("raw", ros::TransportHints(), private_nh);
  image_transport::TransportHints hints_depth("raw", ros::TransportHints(), private_nh/*, depth_image_transport_param*/);
  sub_rgb_  .subscribe(*rgb_it_  , "image_color", 1, hints_rgb);
  sub_depth_.subscribe(*depth_it_, "image"      , 1, hints_depth);
  sub_bbox_ .subscribe(*bbox_nh_ , "darknet_ros/bounding_boxes", 1);

  // wait for camera info topic and register once
  cam_info_ptr_ = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cam_topic_, nh, ros::Duration(3));

  pub_crop_img = nh.advertise<sensor_msgs::Image>("/camera/rgb/image_color_croped", 100);
  NODELET_INFO("ImageCrop: initialize finished.");
}

void ImageCrop::crop(const sensor_msgs::ImageConstPtr& rgb_msg,
                            const sensor_msgs::ImageConstPtr& depth_msg,
                            const darknet_ros_msgs::BoundingBoxesConstPtr& bbox_msg)
{

  for (int bbox_id = 0; bbox_id < bbox_msg->bounding_boxes.size(); bbox_id++)
  {

    darknet_ros_msgs::BoundingBox bbox = bbox_msg->bounding_boxes[bbox_id];

    if (bbox.probability >= 0.5)
    {
      NODELET_INFO("crop start.");
      boost::timer t;

      // クロップ処理を行う
      cv_bridge::CvImagePtr cv_ptr;

      if (rgb_msg->encoding == enc::BGR8)
      {
        try
        {
          cv_ptr = cv_bridge::toCvCopy(rgb_msg, enc::BGR8);
        }
        catch (cv_bridge::Exception &ex)
        {
          ROS_ERROR("error");
          exit(-1);
        }

        cv::Mat &mat = cv_ptr->image;

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

        // generate croped image
        cv::Rect bbox_rect(bbox.xmin, bbox.ymin, bbox.xmax - bbox.xmin, bbox.ymax - bbox.ymin);
        std::cout << "rect: " << bbox.xmin << " " << bbox.ymin << " " << bbox.xmax - bbox.xmin + 10 << " " << bbox.ymax - bbox.ymin + 10 << std::endl;
        cv::Mat crop_img = cv_ptr->image(bbox_rect);
        std_msgs::Header crop_header;
        crop_header.stamp = ros::Time::now();
        crop_header.frame_id = cv_ptr->header.frame_id;
        cv_bridge::CvImage crop_img_bridge(crop_header, "bgr8", crop_img);
        pub_crop_img.publish(crop_img_bridge.toImageMsg());

        // cv::imshow("RGB image", cv_ptr->image);
        // pub_crop_img.publish(cv_ptr->toImageMsg());

        // register detection object
        if (bbox.Class == save_class_name)
          registerObject(rgb_msg, bbox, cv_ptr);

        std::cout << "time: " << t.elapsed() << std::endl;
      }
    }


    NODELET_INFO("crop end");
    std::cout << std::endl << std::endl;
  }

}

void ImageCrop::registerObject(const sensor_msgs::ImageConstPtr& rgb_msg,
                            const darknet_ros_msgs::BoundingBox& bbox,
                            const cv_bridge::CvImagePtr& cv_ptr)
{
  NODELET_INFO("!!!bottle detection!!!");
  DetectionData detected_object(rgb_msg->header, bbox.Class, cv_ptr->image, cam_info_ptr_);
  database.push_back(&detected_object);

  // save image
  const std::string save_path = (boost::format("%simage_%s_prob_%f_%d.png") % save_image_dir % bbox.Class % bbox.probability % counter).str();
  // std::string  save_path = save_image_dir + "image_bottole_prob_" + std::to_string(bbox.probability) + "_" + std::to_string(counter) + ".png";
  std::cout << "save path: " + save_path << std::endl;
  imwrite(save_path, cv_ptr->image);
  counter++;
}

void ImageCrop::count_object(){
  std::cout << "num object: " << database.size() << std::endl;
}


} // namespace bounding_pointcloud

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(bounding_pointcloud::ImageCrop,nodelet::Nodelet);
