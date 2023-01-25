# INS_ESKF_KITTI

**I barely found GPS-IMU fusion localization algorithm using real world dataset on github,most of them are using data generated  from  [gnss-imu-sim](https://github.com/Aceinna/gnss-ins-sim).So I developed [ins_eskf_kitti](https://github.com/leo6862/ins_eskf_kitti),a GPS-IMU fusion localization algorithm using error-state kalman filter based on kitti dataset.I Use the formula that shared By Dr.Gao Xiang in [Zhihu](https://zhuanlan.zhihu.com/p/441182819)**

## Menu

- [INS\_ESKF\_KITTI](#ins_eskf_kitti)
  - [Menu](#menu)
  - [System architecture (Improtant!)](#system-architecture-improtant)
  - [Dependency](#dependency)
  - [Install](#install)
  - [Sample datasets](#sample-datasets)
  - [Run the package](#run-the-package)

## System architecture (Improtant!)
I create a ROS-wrapper(for ROS-noetic),so the core algorithm is dependent from ROS,and easy for other developers to integrate into other platforms or ROS2. And I use ```/kitti/oxts/gps/vel``` and ```orientation in /kitti/oxts/imu/extract``` so I can initialize the whole system.BTW,I also plot the ```position in /kitti/oxts/imu/extract & oirentation in /kitti/oxts/imu/extract``` in rviz as a baseline th show the result of fusion algorithm.


Here in the whole system arcgitecture:

![](images/system%20archetecture.png)
 



## Dependency
ROS-notic

Eigen:
``` 
sudo apt-get install libeigen3-dev 
```
geographiclib :
  ```
  sudo apt-get install libgeographic-dev
  ```
  
  glog : 
  ```
  git clone https://github.com/google/glog.git
  cd glog
  mkdir build 
  cd build
  cmake ..
  make
  ```

  yaml-cpp:
  ```
  sudo apt-get install libyaml-cpp-dev
  ```

## Install

Use the following commands to download and compile the package.
**Before compiling the project , you have to edit the ```include/global_definition.h``` change the PROJECT_PATH to yours.**
Then



```
cd ~/catkin_ws/src
git clone https://github.com/leo6862/ins_eskf_kitti.git
cd ..
catkin_make
```

## Sample datasets
Using a Kitti Dataset which has imu data frequency higher than 100HZ.
here's a dataset from mine.You can download it from  [baidu net disk](https://pan.baidu.com/s/15V587gC7cC6YZp_250ShEQ) 提取码: wtu9.

## Run the package

1. Run the launch file:
```
roslaunch ins_eskf ins_eskf.launch
```

2. Play existing bag files:
```
rosbag play your-bag.bag
```

