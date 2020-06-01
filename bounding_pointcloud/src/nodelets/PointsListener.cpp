#include <limits>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/CameraInfo.h"
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>


#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/CameraInfo.h"
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>

#include "pcl_ros/point_cloud.h"
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// nodeletの書き方の作法として、namespaceを用いる
// クラスはnodelet::Nodeletを継承
// NODELET_INFOを用いる
// 起動時にonInit()が呼ばれるので、ここに機能を実装する。
// onInit()で無限ループを書くと他のノードが呼ばれなくなるので厳禁
// ノードハンドラはgetNodeHandle(), getPrivateNodeHandle()で呼ぶ
// PUUGINLIBを用いているので、最後にPLUGINLIB_EXPORT_CLASSを書く。第一引数はクラス名、第二引数はベースクラス名
namespace bounding_pointcloud{
class PointsListener : public nodelet::Nodelet
{
public:
    virtual void onInit();
      void callback_pc(const sensor_msgs::PointCloud2ConstPtr& msg);
    // void callback_pc(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg);

    std_msgs::Header createHeader(const std::string &frame_id);

private:
    ros::NodeHandle nh_;

    ros::Subscriber pc_sub;
    ros::Publisher pc_pub;

    const char *img_sub_topic = "/camera/rgb/image_color";
    const char *img_pub_topic = "/camera/republish/rgb/image_color";
    const char *cam_info_sub_topic = "/camera/rgb/camera_info";
    const char *cam_info_pub_topic = "/camera/republish/rgb/camera_info";
    const char *pc_sub_topic = "/depth_registered/points";
    const char *pc_pub_topic = "/coeff";

    sensor_msgs::PointCloud2 pc;
    uint32_t num_sub_pc = 0;
};

void PointsListener::onInit()
{
    NODELET_INFO("Listener Init");
    nh_ = getNodeHandle();
    pc_sub = nh_.subscribe(pc_sub_topic, 10, &PointsListener::callback_pc, this);
    // ros::Subscriber sub = nh_.subscribe<pcl::PointCloud<pcl::PointXYZRGB>>(pc_sub_topic, 10, &PointsListener::callback_pc, this);

      pc_pub = nh_.advertise<pcl_msgs::ModelCoefficients>(pc_pub_topic, 100);

    std::cout << "class PointsListener contructor." << std::endl;
}

void PointsListener::callback_pc(const sensor_msgs::PointCloud2ConstPtr &input_msg)
// void PointsListener::callback_pc(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)
{

    // std::cout << "stamp:" << msg->header.stamp << std::endl;
    // std::cout << "point size:" << msg->points.size() << std::endl;
    // std::cout << "stamp:" << msg->header.stamp << std::endl;

    // for(int i=0; (i<5) && (i<msg->points.size()); i++)
    // {   std::cout << "point:"   << msg->points[i].x << " "
    //                             << msg->points[i].y << " "
    //                             << msg->points[i].z << " "
    //                             << msg->points[i].r << " "
    //                             << msg->points[i].g << " "
    //                             << msg->points[i].b
    //                             << std::endl;
    // }
    // std::cout << std::endl;


//   pc = msg;
// //   pc.header = createHeader(msg.header.frame_id);
// //   pc.header.seq = num_sub_pc;
// //   pc_pub.publish(pc);

//   num_sub_pc++;

//   std::cout << "msg header: " << msg.header.stamp << " " << msg.header.frame_id << " " << msg.header.seq << std::endl;
//   std::cout << "after createHeader: " << pc.header.stamp << " " << pc.header.frame_id << " " << pc.header.seq << std::endl;
//   std::cout << "pc callback: " << num_sub_pc << std::endl << std::endl;



  // pcl tutorial
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg (*input_msg, cloud);

  // print
    std::cout << "stamp:" << cloud.header.stamp << std::endl;
    std::cout << "point size:" << cloud.points.size() << std::endl;
    std::cout << "width:" << cloud.width << std::endl;
    std::cout << "height:" << cloud.height << std::endl;
    // std::cout << "fields:" << cloud.fields << std::endl;


    for(int i=0; i<cloud.points.size(); i++)
    {
        // std::cout << cloud.points[i] << std::endl;

        if( !isnan(cloud.points[i].x))
        {
            std::cout << "point " << i << "  : "
                      << cloud.points[i].x << " "
                      << cloud.points[i].y << " "
                      << cloud.points[i].z << " "
                      << cloud.points[i].rgb << " "
                      << int(cloud.points[i].r) << " "
                      << int(cloud.points[i].g) << " "
                      << int(cloud.points[i].b)
                      << std::endl;

            // uint32_t rgb = *reinterpret_cast<int *>(&cloud.points[i].rgb);
            // uint8_t r = (rgb >> 16) & 0x0000ff;
            // uint8_t g = (rgb >> 8)  & 0x0000ff;
            // uint8_t b = (rgb)       & 0x0000ff;
            // std::cout << "r g b: " << r << " " << g << " " << b << std::endl;
        }
    }
    std::cout << std::endl;

  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud.makeShared ());
  seg.segment (inliers, coefficients);

  // Publish the model coefficients
  pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(coefficients, ros_coefficients);
  pc_pub.publish (ros_coefficients);

}

std_msgs::Header PointsListener::createHeader(const std::string &frame_id_)
{
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = frame_id_;

  return header;
}

}  // namespace bounding_pointcloud
PLUGINLIB_EXPORT_CLASS(bounding_pointcloud::PointsListener, nodelet::Nodelet)