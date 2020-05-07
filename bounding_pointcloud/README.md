# bounding_pointcloud

## 概要

- RGBDデータがROSシステム上に存在し、物体検知が行われた場合、物体が検知されたときの点群を物体検出のバウンディングボックスに合わせて切り取り、オクトマップで表現する。

## ノード

- bounding_pointcloud_node
  - TimeSynchronizerを用いているノード。失敗作。
- synchronizer_node
  - ApproximateTimeSynchronizerを用いてトピックの同期をとるノード
- ScanPublisher_node
  - ApproximateTimeSynchronizerの挙動を確かめるためにlaserscanのトピックをパブリッシュさせるテスト用のノード
- debug_node
  - 特定のトピックをサブスクライブし、そのトピックのヘッダをprintするノード

- nodelet
  - nodelet_test.launchで起動。データセットのヘッダの時刻が数年前で合わないので、現在時刻になおしてパブリッシュするノード
  - rosbagを--clockで再生し、rosparam set /use_sim_time trueとすることで時刻の書き換えを不要にできる。

## 使いかた

- roscore
- rosbag play -l  [dataset_name] --clock
  - rosparam set /use_sim_time true
- roslaunch darknet_ros darknet_ros.launch image:=/camera/republish/rgb/image_color
- rosrun bounding_pointcloud synchronizer_noide

## 今後の方針

- 同期が取れることがわかったので、物体検出した点群だけをパブリッシュする
- 同期を取ってパブリッシュするノードをnodeletにする。
- オクトマップに表現してみる。