#include "bounding_pointcloud/ObjectDatabase.hpp"
#include <pluginlib/class_list_macros.h>

using namespace std;

// nodeletの書き方の作法として、namespaceを用いる
// クラスはnodelet::Nodeletを継承
// NODELET_INFOを用いる
// 起動時にonInit()が呼ばれるので、ここに機能を実装する。
// onInit()で無限ループを書くと他のノードが呼ばれなくなるので厳禁
// ノードハンドラはgetNodeHandle(), getPrivateNodeHandle()で呼ぶ
// PUUGINLIBを用いているので、最後にPLUGINLIB_EXPORT_CLASSを書く。第一引数はクラス名、第二引数はベースクラス名
namespace bounding_pointcloud{

void ObjectDatabase::onInit()
{
  NODELET_INFO("Listener Init");
  nh_ = getNodeHandle();
  sub_ = nh_.subscribe("chatter", 10, &ObjectDatabase::chatter_callback, this);

//   img_sub = nh_.subscribe(img_sub_topic, 10, &ObjectDatabase::callback_img, this);
//   cam_info_sub = nh_.subscribe(cam_info_sub_topic, 10, &ObjectDatabase::callback_cam_info, this);
//   pc_sub = nh_.subscribe(pc_sub_topic, 10, &ObjectDatabase::callback_pc, this);

//   img_pub = nh_.advertise<sensor_msgs::Image>(img_pub_topic, 100);
//   cam_info_pub = nh_.advertise<sensor_msgs::CameraInfo>(cam_info_pub_topic, 100);
//   pc_pub = nh_.advertise<sensor_msgs::PointCloud2>(pc_pub_topic, 100);

  std::cout << "class ObjectDatabase contructor." << std::endl;
}

}  // namespace bounding_pointcloud
PLUGINLIB_EXPORT_CLASS(bounding_pointcloud::ObjectDatabase, nodelet::Nodelet)