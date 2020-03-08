# packages_wheelchair
## Overview
このリポジトリは東京都市大学高機能機械制御研究室電動車椅子班 Robot Operating System(ROS)でのパッケージを掲載しています．
以下にインストール方法を記載するので，その手順に従ってパッケージをインストールしてください．  
また，このインストールでは*Ubuntu 18.04LTS*及び*ROS melodic*を対象に紹介しています．  

## ROSインストール
まず，[Ubuntu install of ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)から*ROS Melodic*のインストールを行ってください．  
インストール終了後，端末上で`$ sudo reboot`によって再起動をしてから，以下のコマンドを端末に入力してください．  
```
$ sudo apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
$ sudo apt install -y ros-melodic-gazebo-ros-control
$ sudo apt install -y ros-melodic-ros-control ros-melodic-ros-controllers
$ sudo apt install -y ros-melodic-navigation 
$ sudo apt install -y ros-melodic-realsense-camera
$ sudo apt install -y libpcap0.8-dev
$ sudo apt install -y libncurses5-dev
```
## Package 追加
続いて，ROSで使用するプログラムを管理・ビルドするために必要なcatkin workspaceを作成します．  
[Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)にあるように以下のコマンドを端末上で実行してください．
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
```
ワークスペースは空（srcフォルダにパッケージが無く,ただCMakeLists.txtのリンクがあるだけ）です.  
しかし，以下の手順でcatkin_wsをビルドすることができます．
```
$ cd ~/catkin_ws/
$ catkin_make
```
特にエラーが発生しなければ次に，対応するパッケージを追加する作業を実施します．  
それぞれのパッケージはすべてgithubにあり，それぞれを端末上で次のように実行することでcloneすることが可能です．
まず`~/catkin_ws/src`内で電動車椅子班用パッケージをcloneします． 
```
$ cd ~/catkin_ws/src
$ git clone --recursive https://github.com/g1412049/packages_wheelchair.git
```
次に，velodyne LiDAR用のパッケージををcloneします． 
```
$ git clone https://github.com/ros-drivers/velodyne.git
$ git clone https://bitbucket.org/DataspeedInc/velodyne_simulator.git
```
さらに[Intel&reg; RealSense&trade; Devices](https://github.com/IntelRealSense/realsense-ros)のcloneを実行してください．
```
$ git clone https://github.com/IntelRealSense/realsense-ros.git
$ cd realsense-ros/
$ git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1`
$ cd ~/catkin_ws/src
```
最後に NaturalPoint OptiTrack motion capture system用[NatNet 3 ROS driver](https://github.com/mje-nz/natnet_ros)をcloneします． 
```
$ git clone --recursive https://github.com/mje-nz/natnet_ros.git
$ cd ..
$ rosdep install -y --from-paths src --ignore-src
$ catkin_make
$ source devel/setup.bash
```
以上がインストール及び設定手順です．  
エラーが発生した際はそれぞれの文章を読んで足りないパッケージの追加やエラーの改善を行ってください．

## 更新情報
- 初版(2020/03/08): 基本事項の追加
