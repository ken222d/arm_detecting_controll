# arm_detecting_controll

## パッケージの説明
このパッケージには, 知能ロボットコンテストに向けたシミュレーション環境とロボットの動作プログラムがあります. 

## 動作説明
- カメラにボールが映るまでライントレースを行う. 
- ボールを検出したらライントレースを終了し目標ボールに接近. 
- 目標ボールとの距離が近くなったら, アームを制御してボールを確保. 

## 使い方
```bash
cd ~/ros2_ws/src
git clone https://github.com/ken222d/arm_detecting_controll
```

- アームの動作
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash

#ターミナル１
$ ros2 launch arm_detecting_controll rdc_arm.launch.py #Gazebo起動
#ターミナル２
$ ros2 run arm_detecting_controll linetrace_controll
#ターミナル３
$ ros2 run arm_detecting_controll arm_controll
#ターミナル4
$ ros2 run arm_detecting_controll multi_color_ball_detector #GUI起動
#ターミナル5
$ ros2 run arm_detecting_controll ball_follower
```

## デモ動画
- ライントレース及びボール検出、接近

[![Demo Video](https://youtube.com/vi/m3qrZkw8Mq8/0.jpg)](https://youtu.be/m3qrZkw8Mq8)

- アーム動作

[![Demo Video](https://img.youtube.com/vi/g60Mg9bKciM/0.jpg)](https://youtu.be/g60Mg9bKciM)

## 使用パッケージ
- gazebo_ros2_control
[URL](https://github.com/ros-controls/gazebo_ros2_control)
- gazebo_ros2_link_attacher
[URL](https://github.com/davidorchansky/gazebo_ros_link_attacher/tree/humble-devel)


## 使用ノード
- arm_controll アームの動作制御ノード
- linetrace_controll付きアーム動作制御ノード
- multi_color_ball_detector カメラノード
- ball_follower 車輪の制御ノード

## 使用ソフトウェア
- ROS 2 Humble
- Gazebo Classic

## 環境
- Ubuntu22.04

## 使用した関連ツール（開発・検証に使用）

本プロジェクトの開発・検証にあたって, 以下のOSSツールをUbuntu上で使用しました（本リポジトリには含まれていません）:

- [gazebo_ros2_link_attacher](https://github.com/davidorchansky/gazebo_ros_link_attacher/tree/humble-devel): Gazebo上でリンクを接続するためのROS 2パッケージ
- [gazebo_ros_control](https://github.com/ros-controls/gazebo_ros2_control): GazeboとROS 2のインターフェース

これらのコードは本リポジトリに含まれていませんが, 検証・開発において活用しました. 


## ライセンスと著作権
- このソフトウェアパッケージは, 3条項BSDライセンスの下, 再頒布および使用が許可されます.
- © 2025 Kenta Ishizeki 

