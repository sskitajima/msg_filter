  /*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
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
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include "darknet_ros_msgs/BoundingBoxes.h"
#include "bounding_pointcloud/depth_traits.h"
#include "bounding_pointcloud_msgs/CropImage.h"
#include "bounding_pointcloud/ObjectDatabase.hpp"

namespace bounding_pointcloud {

using namespace message_filters::sync_policies;
namespace enc = sensor_msgs::image_encodings;
typedef boost::shared_ptr< ::bounding_pointcloud_msgs::CropImage const > CropImageConstPtr;

class CropImagePointCloud : public nodelet::Nodelet
{
  ros::NodeHandlePtr rgb_nh_;
  boost::shared_ptr<image_transport::ImageTransport> rgb_it_, depth_it_;
  
  //// TODO ////
  // poseをsub
  // CropImageをsub

  //// このノードですること
  // CropImageとpose、camera_infoを同期する
  // CropImageからpointsを作る。パブリッシュする
  // pointsはClassによって色を変える
  // poseとCropImageの情報をデータベースに登録する

  // Subscriptions
  message_filters::Subscriber<bounding_pointcloud_msgs::CropImage> sub_cropImage_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_pose_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
  typedef ApproximateTime<bounding_pointcloud_msgs::CropImage, sensor_msgs::CameraInfo, geometry_msgs::PoseStamped> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  boost::shared_ptr<Synchronizer> sync_;

  // Publications
  boost::mutex connect_mutex_;
  typedef sensor_msgs::PointCloud2 PointCloud;
  ros::Publisher pub_point_cloud_;

  image_geometry::PinholeCameraModel model_;

  const char* topic_name_cropImage = "/bounding_pointcloud_msgs/CropImage";
  const char* topic_name_pose = "/orb_slam2_rgbd/pose";

  virtual void onInit();

  void connectCb();

  void imageCb(const bounding_pointcloud::CropImageConstPtr& cropImage_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg,
               const geometry_msgs::PoseStampedConstPtr& pose_msg);

  template<typename T>
  void convert(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::ImageConstPtr& rgb_msg,
               const PointCloud::Ptr& cloud_msg,
               int red_offset, int green_offset, int blue_offset, int color_step);
};

void CropImagePointCloud::onInit()
{
  ros::NodeHandle& nh         = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  rgb_nh_.reset( new ros::NodeHandle(nh, "camera/rgb") );
  ros::NodeHandle depth_nh(nh, "points_croped");

  // Read parameters
  int queue_size;
  bool use_exact_sync;
  private_nh.param("queue_size", queue_size, 10000000);
  // private_nh.param("exact_sync", use_exact_sync, false);

  sync_.reset(new Synchronizer(SyncPolicy(queue_size), sub_cropImage_, sub_info_, sub_pose_));
  sync_->registerCallback(boost::bind(&CropImagePointCloud::imageCb, this, _1, _2, _3));

  // Monitor whether anyone is subscribed to the output
  // ros::SubscriberStatusCallback connect_cb = boost::bind(&CropImagePointCloud::connectCb, this);
  // // Make sure we don't enter connectCb() between advertising and assigning to pub_point_cloud_
  // boost::lock_guard<boost::mutex> lock(connect_mutex_);
  // pub_point_cloud_ = depth_nh.advertise<PointCloud>("points", 1, connect_cb, connect_cb);
  pub_point_cloud_ = depth_nh.advertise<PointCloud>("points", 100);

  // rgb uses normal ros transport hints.
  sub_info_.subscribe(*rgb_nh_, "camera_info", 1);
  sub_cropImage_.subscribe(private_nh, topic_name_cropImage, 1);
  sub_pose_.subscribe(private_nh, topic_name_pose, 1);

  NODELET_INFO("initialize finished. crop_point_cloud");
}

// Handles (un)subscribing when clients (un)subscribe
void CropImagePointCloud::connectCb()
{
  NODELET_INFO("connectCb called.");
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_point_cloud_.getNumSubscribers() == 0)
  {
    sub_cropImage_.unsubscribe();
    sub_pose_.unsubscribe();
    sub_info_ .unsubscribe();
  }
  else if (!sub_cropImage_.getSubscriber())
  {
    NODELET_INFO("!sub_cropImage.getsubscriber()");
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    // rgb uses normal ros transport hints.
    sub_info_ .subscribe(*rgb_nh_,   "camera_info",      1);
    sub_cropImage_.subscribe(private_nh, topic_name_cropImage, 1);
    sub_pose_.subscribe(private_nh, topic_name_pose, 1);
  }
}

void CropImagePointCloud::imageCb(const bounding_pointcloud::CropImageConstPtr& cropImage_msg,
                                      const sensor_msgs::CameraInfoConstPtr& info_msg,
                                      const geometry_msgs::PoseStampedConstPtr& pose_msg)

{
  NODELET_INFO("!!!!!!!!CropImagePointCloud:imageCb called.!!!!!!!!!!!!");
  sensor_msgs::ImageConstPtr depth_msg = sensor_msgs::ImageConstPtr(&(cropImage_msg->rgb_image));
  sensor_msgs::ImageConstPtr rgb_msg_in = sensor_msgs::ImageConstPtr(&(cropImage_msg->depth_image));


  // Check for bad inputs
  if (depth_msg->header.frame_id != rgb_msg_in->header.frame_id)
  {
    NODELET_ERROR_THROTTLE(5, "Depth image frame id [%s] doesn't match RGB image frame id [%s]",
                           depth_msg->header.frame_id.c_str(), rgb_msg_in->header.frame_id.c_str());
    return;
  }

  // Update camera model
  model_.fromCameraInfo(info_msg);

  // Check if the input image has to be resized
  sensor_msgs::ImageConstPtr rgb_msg = rgb_msg_in;
  if (depth_msg->width != rgb_msg->width || depth_msg->height != rgb_msg->height)
  {
    sensor_msgs::CameraInfo info_msg_tmp = *info_msg;
    info_msg_tmp.width = depth_msg->width;
    info_msg_tmp.height = depth_msg->height;
    float ratio = float(depth_msg->width)/float(rgb_msg->width);
    info_msg_tmp.K[0] *= ratio;
    info_msg_tmp.K[2] *= ratio;
    info_msg_tmp.K[4] *= ratio;
    info_msg_tmp.K[5] *= ratio;
    info_msg_tmp.P[0] *= ratio;
    info_msg_tmp.P[2] *= ratio;
    info_msg_tmp.P[5] *= ratio;
    info_msg_tmp.P[6] *= ratio;
    model_.fromCameraInfo(info_msg_tmp);

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(rgb_msg, rgb_msg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv_bridge::CvImage cv_rsz;
    cv_rsz.header = cv_ptr->header;
    cv_rsz.encoding = cv_ptr->encoding;
    cv::resize(cv_ptr->image.rowRange(0,depth_msg->height/ratio), cv_rsz.image, cv::Size(depth_msg->width, depth_msg->height));
    if ((rgb_msg->encoding == enc::RGB8) || (rgb_msg->encoding == enc::BGR8) || (rgb_msg->encoding == enc::MONO8))
      rgb_msg = cv_rsz.toImageMsg();
    else
      rgb_msg = cv_bridge::toCvCopy(cv_rsz.toImageMsg(), enc::RGB8)->toImageMsg();

    //NODELET_ERROR_THROTTLE(5, "Depth resolution (%ux%u) does not match RGB resolution (%ux%u)",
    //                       depth_msg->width, depth_msg->height, rgb_msg->width, rgb_msg->height);
    //return;
  } else
    rgb_msg = rgb_msg_in;

  // Supported color encodings: RGB8, BGR8, MONO8
  int red_offset, green_offset, blue_offset, color_step;
  if (rgb_msg->encoding == enc::RGB8)
  {
    red_offset   = 0;
    green_offset = 1;
    blue_offset  = 2;
    color_step   = 3;
  }
  else if (rgb_msg->encoding == enc::BGR8)
  {
    red_offset   = 2;
    green_offset = 1;
    blue_offset  = 0;
    color_step   = 3;
  }
  else if (rgb_msg->encoding == enc::MONO8)
  {
    red_offset   = 0;
    green_offset = 0;
    blue_offset  = 0;
    color_step   = 1;
  }
  else
  {
    try
    {
      rgb_msg = cv_bridge::toCvCopy(rgb_msg, enc::RGB8)->toImageMsg();
    }
    catch (cv_bridge::Exception& e)
    {
      NODELET_ERROR_THROTTLE(5, "Unsupported encoding [%s]: %s", rgb_msg->encoding.c_str(), e.what());
      return;
    }
    red_offset   = 0;
    green_offset = 1;
    blue_offset  = 2;
    color_step   = 3;
  }

  // Allocate new point cloud message
  PointCloud::Ptr cloud_msg(new PointCloud);
  cloud_msg->header = depth_msg->header; // Use depth image time stamp

  cloud_msg->height = depth_msg->height;
  cloud_msg->width = depth_msg->width;
  NODELET_INFO("cloud_msg height: %d, width: %d", cloud_msg->height, cloud_msg->width);

  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb"); // xyzrgbのフィールドにcloud_msgを登録する。

  // uint16_t で、チャンネルが1つ-> 16UC1
  if (depth_msg->encoding == enc::TYPE_16UC1)
  {
    NODELET_INFO("encoding: TYPE_16_UC1");
    convert<uint16_t>(depth_msg, rgb_msg, cloud_msg, red_offset, green_offset, blue_offset, color_step);
  }
  // float32で、チャンネルが1つ->32FC1
  else if (depth_msg->encoding == enc::TYPE_32FC1)
  {
    NODELET_INFO("encoding: TYPE_32_FC1");
    convert<float>(depth_msg, rgb_msg, cloud_msg, red_offset, green_offset, blue_offset, color_step);
  }
  else
  {
    NODELET_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
    return;
  }

  pub_point_cloud_.publish(cloud_msg);

  std::cout << std::endl;
}

// cloud_msgの中に出力が格納されていく
// T...uint32_またはfloat
template<typename T>
void CropImagePointCloud::convert(const sensor_msgs::ImageConstPtr& depth_msg,
                                      const sensor_msgs::ImageConstPtr& rgb_msg,
                                      const PointCloud::Ptr& cloud_msg,
                                      int red_offset, int green_offset, int blue_offset, int color_step)
{
  NODELET_INFO("convert function start.");

  // Use correct principal point from calibration
  float center_x = model_.cx();
  float center_y = model_.cy();

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  // principal point ... 主点
  double unit_scaling = DepthTraits<T>::toMeters( T(1) );
  float constant_x = unit_scaling / model_.fx();
  float constant_y = unit_scaling / model_.fy();
  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  
  const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);
  const uint8_t* rgb = &rgb_msg->data[0];
  int rgb_skip = rgb_msg->step - rgb_msg->width * color_step;

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud_msg, "b");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_a(*cloud_msg, "a");

  for (int v = 0 /*bbox.ymin*/; v < depth_msg->height /*cloud_msg->height*/ /*bbox.ymax*/; ++v, depth_row += row_step, rgb += rgb_skip)
  {

    for (int u = 0 /*bbox.xmin*/; u < depth_msg->width /*cloud_msg->width*/ /*bbox.xmax*/; ++u, rgb += color_step, ++iter_x, ++iter_y, ++iter_z, ++iter_a, ++iter_r, ++iter_g, ++iter_b)
    {

      // NODELET_INFO("v: %d, u: %d", v, u);
      T depth = depth_row[u];

      // Check for invalid measurements
      if (!DepthTraits<T>::valid(depth))
      {
        *iter_x = *iter_y = *iter_z = bad_point;
      }
      else
      {
        // Fill in XYZ
        *iter_x = (u - center_x) * depth * constant_x;
        *iter_y = (v - center_y) * depth * constant_y;
        *iter_z = DepthTraits<T>::toMeters(depth);
      }

      // Fill in color
      *iter_a = 255;
      *iter_r = rgb[red_offset];
      *iter_g = rgb[green_offset];
      *iter_b = rgb[blue_offset];
    }
  }
}

} // namespace bounding_pointcloud

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(bounding_pointcloud::CropImagePointCloud,nodelet::Nodelet);
