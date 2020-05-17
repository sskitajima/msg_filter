#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/CameraInfo.h"
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>

// nodeletの書き方の作法として、namespaceを用いる
// クラスはnodelet::Nodeletを継承
// NODELET_INFOを用いる
// 起動時にonInit()が呼ばれるので、ここに機能を実装する。
// onInit()で無限ループを書くと他のノードが呼ばれなくなるので厳禁
// ノードハンドラはgetNodeHandle(), getPrivateNodeHandle()で呼ぶ
// PUUGINLIBを用いているので、最後にPLUGINLIB_EXPORT_CLASSを書く。第一引数はクラス名、第二引数はベースクラス名
namespace bounding_pointcloud{
class plugin_nodelet_listener : public nodelet::Nodelet
{
public:
  virtual void onInit();
  void chatter_callback(const std_msgs::String &string_msg);
  
  void callback_img(const sensor_msgs::Image& msg);
  void callback_cam_info(const sensor_msgs::CameraInfo& msg);
  void callback_pc(const sensor_msgs::PointCloud2& msg);
  
  std_msgs::Header createHeader(const std::string& frame_id);

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  ros::Subscriber img_sub;
  ros::Subscriber cam_info_sub;
  ros::Subscriber pc_sub;

  ros::Publisher img_pub;
  ros::Publisher cam_info_pub;
  ros::Publisher pc_pub;

  const char* img_sub_topic = "/camera/rgb/image_color";
  const char* img_pub_topic = "/camera/republish/rgb/image_color";
  const char* cam_info_sub_topic = "/camera/rgb/camera_info";
  const char* cam_info_pub_topic = "/camera/republish/rgb/camera_info";
  const char* pc_sub_topic = "/camera/rgb/points";
  const char* pc_pub_topic = "/camera/republish/rgb/points";

  sensor_msgs::Image img;
  sensor_msgs::CameraInfo cam_info;
  sensor_msgs::PointCloud2 pc;

  uint32_t num_sub_img = 0;
  uint32_t num_sub_caminfo = 0;
  uint32_t num_sub_pc = 0;
};

void plugin_nodelet_listener::onInit()
{
  NODELET_INFO("Listener Init");
  nh_ = getNodeHandle();
  sub_ = nh_.subscribe("chatter", 10, &plugin_nodelet_listener::chatter_callback, this);

  img_sub = nh_.subscribe(img_sub_topic, 10, &plugin_nodelet_listener::callback_img, this);
  cam_info_sub = nh_.subscribe(cam_info_sub_topic, 10, &plugin_nodelet_listener::callback_cam_info, this);
  pc_sub = nh_.subscribe(pc_sub_topic, 10, &plugin_nodelet_listener::callback_pc, this);

  img_pub = nh_.advertise<sensor_msgs::Image>(img_pub_topic, 100);
  cam_info_pub = nh_.advertise<sensor_msgs::CameraInfo>(cam_info_pub_topic, 100);
  pc_pub = nh_.advertise<sensor_msgs::PointCloud2>(pc_pub_topic, 100);

  std::cout << "class plugin_nodelet_listener contructor." << std::endl;
}

void plugin_nodelet_listener::chatter_callback(const std_msgs::String &string_msg)
{
  NODELET_INFO("sample callback");
  NODELET_INFO("recieve: %s", string_msg.data.c_str());
}

void plugin_nodelet_listener::callback_img(const sensor_msgs::Image &msg)
{
  img = msg;
  img.header = createHeader(msg.header.frame_id);
  img.header.seq = num_sub_img;
  img_pub.publish(img);

  num_sub_img++;
  std::cout << "img callback: " << num_sub_img << std::endl;
}

void plugin_nodelet_listener::callback_cam_info(const sensor_msgs::CameraInfo &msg)
{
  cam_info = msg;
  cam_info.header = createHeader(msg.header.frame_id);
  cam_info.header.seq = num_sub_caminfo;
  cam_info_pub.publish(cam_info);

  num_sub_caminfo++;
  std::cout << "caminfo callback: " << num_sub_caminfo << std::endl;
}

void plugin_nodelet_listener::callback_pc(const sensor_msgs::PointCloud2 &msg)
{
  pc = msg;
  pc.header = createHeader(msg.header.frame_id);
  pc.header.seq = num_sub_pc;
  pc_pub.publish(pc);

  num_sub_pc++;

  std::cout << "msg header: " << msg.header.stamp << " " << msg.header.frame_id << " " << msg.header.seq << std::endl;
  std::cout << "after createHeader: " << pc.header.stamp << " " << pc.header.frame_id << " " << pc.header.seq << std::endl;
  std::cout << "pc callback: " << num_sub_pc << std::endl;
}

std_msgs::Header plugin_nodelet_listener::createHeader(const std::string &frame_id_)
{
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = frame_id_;

  return header;
}

}  // namespace bounding_pointcloud
PLUGINLIB_EXPORT_CLASS(bounding_pointcloud::plugin_nodelet_listener, nodelet::Nodelet)