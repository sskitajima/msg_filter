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
- nodelet/point_cloud_xyzrgb
  - rosのパッケージdepth_image_procの改良
  - boundingbox, image, depth_image, camera_infoの情報を物体が検出されたときの点群のみを選んでパブリッシュする。

## 使いかた

- ```roscore```

- ```
  rosparam set /use_sim_time true
  rosbag play -l  [dataset_name] --clock
  ```
  
  - pointsも含めてあるbagファイルを使う。rgbd_benckmark_toolsのadd_pointclouds_to_bagfile.pyを用いる。pointsがないbagファイルを再生してもなぜか動悸されない
  
- roslaunch bounding_pointcloud bounding_pointcloud.launch

- roslaunch bounding_pointcloud nodelet_test.launch







## 今後の方針

- 同期が取れることがわかったので、物体検出した点群だけをパブリッシュする
- 同期を取ってパブリッシュするノードをnodeletにする。
- オクトマップに表現してみる。