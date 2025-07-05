# arm_detecting_controll

## パッケージの説明
このパッケージには, 知能ロボットコンテストのシミュレーション環境とロボットの動作プログラムがあります. 

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
$ ros2 run arm_detecting_controll arm_controll
#ターミナル３
$ ros2 run arm_detecting_controll multi_color_ball_detector #GUI起動
#ターミナル４
$ ros2 run arm_detecting_controll ball_follower
```

- Attach付きアーム動作
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash

#ターミナル１
$ ros2 launch arm_detecting_controll rdc_arm.launch.py #Gazebo起動
#ターミナル２
$ ros2 run arm_detecting_controll attach_arm_ball
#ターミナル３
$ ros2 run arm_detecting_controll multi_color_ball_detector #GUI起動
#ターミナル４
$ ros2 run arm_detecting_controll ball_follower
```

## Issue
アームとボールが接触して時に, ボールをアームに付着して, アームを動かしたいが、付着した後の動作が行われない. （実際は機体の後側に手先がくるようにelbow_linkを動かす. ）
## デモ動画
- アーム動作

[![Demo Video](https://img.youtube.com/vi/grbSSv7qHhg/0.jpg)](https://youtu.be/grbSSv7qHhg)

- Attach付きアーム動作（できていない）

[![Demo Video](https://img.youtube.com/vi/po6zorUj3AM/0.jpg)](https://youtu.be/po6zorUj3AM)

## 使用パッケージ
- gazebo_ros2_control
[URL](https://github.com/ros-controls/gazebo_ros2_control)
- gazebo_ros2_link_attacher
[URL](https://github.com/yliu213/gazebo_ros2_link_attacher)


## 使用ノード
- arm_controll アームの動作制御ノード
- attach_arm_ball Attach付きアーム動作制御ノード
- multi_color_ball_detector カメラノード
- ball_follower 車輪の制御ノード

## 使用ソフトウェア
- ROS 2 Humble
- Gazebo Classic

## 環境
- Ubuntu22.04

## 使用した関連ツール（開発・検証に使用）

本プロジェクトの開発・検証にあたって, 以下のOSSツールをUbuntu上で使用しました（本リポジトリには含まれていません）:

- [gazebo_ros2_link_attacher](https://github.com/yliu213/gazebo_ros2_link_attacher): Gazebo上でリンクを接続するためのROS 2パッケージ
- [gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs): GazeboとROS 2のインターフェース

これらのコードは本リポジトリに含まれていませんが, 検証・開発において活用しました. 


## ライセンスと著作権
- このソフトウェアパッケージは, 3条項BSDライセンスの下, 再頒布および使用が許可されます.
- © 2025 Kenta Ishizeki 

