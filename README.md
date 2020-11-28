# 1. slam2d
* scan match
* correspondence
* map update


## 1.1 scan match
ICP/PLICP method is used for scan match estimation, and ceres is used as the nonlinear solve with point to line contraints.

## 1.2 correspondence
KNN method of pcl lib is used to find point correspondence

## 1.3 map update
OpenCV function line and circle is used to update grid map.


![slam2d](https://github.com/libing64/slam2d/blob/master/image/slam2d.png) 

# 2. Building and Running
## 2.1 Building
```
cd catkin_ws/src
git clone git@github.com:libing64/slam2d.git
cd ..
catkin_make -DCATKIN_WHITELIST_PACKAGES="slam2d"
```


## 2.2 Running
```
soure devel/setup.bash
roslaunch slam2d slam2d.launch
```

play your rosbag
```
rosbag play youbag
```
cartographer dataset:

https://google-cartographer-ros.readthedocs.io/en/latest/data.html

## 2.3 Enviroment
* Ubuntu20.04 
* ros noetic
* OpenCV
* pcl
* Eigen
* ceres

## 2.4 result

# 3. TODO
* scan-to-map match
* keyframe selection to reduce drift
* dynamic range 2d map

# 4. Reference
* An ICP variant using a point-to-line metric
* A flexible and Scalable SLAM System with Full 3D motion Estimation
* Real-Time Loop Closure in 2D LIDAR SLAM