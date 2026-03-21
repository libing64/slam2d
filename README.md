# 1. slam2d
* scan match
* correspondence
* scan-to-map match
* map update

![gif](https://github.com/libing64/slam2d/blob/master/image/slam2d.gif) 

## 1.1 scan-to-scan match
ICP/PLICP method is used for scan-to-scan match, and ceres is used as the nonlinear solver with point-to-line contraints.

## 1.2 correspondence
KNN method of pcl lib is used to find point correspondence


## 1.3 scan-to-map match
A Branch-and-Bound (B2B) algorithm with a precomputed multi-resolution Grid Pyramid is used for robust global/local scan-to-map matching. Bilinear interpolation provides sub-pixel accuracy during map scoring.

## 1.4 map update
A custom 2D Voxel Map (Occupancy Grid) is maintained using log-odds. The map is updated using a custom implementation of Bresenham's line algorithm, replacing the previous OpenCV `cv::Mat` approach for improved accuracy and flexibility. Keyframe selection is implemented to reduce drift.


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

![rosgraph](https://github.com/libing64/slam2d/blob/master/image/slam2d_rosgraph.png) 

## 2.3 Enviroment
* Ubuntu20.04 
* ros noetic
* OpenCV
* pcl
* Eigen
* ceres

# 3. TODO
- [x] how to reduce the drift (added keyframe selection to update map less frequently)
- [x] how to improve the mapping accurcy (implemented VoxelMap and B2B, fixed Ceres parameter block size bug)
- [x] scan-to-map match
- [x] keyframe selection to reduce drift (implemented with distance/angle thresholds)
- [ ] dynamic range 2d map
- [x] search local optimal (Branch-and-Bound matcher)
- [ ] record gif with byzana-record
- [ ] sparse pose adjust
- [ ] loop closure




# 4. Comparison of 2D SLAM
## 4.1 Hector SLAM
* scan-to-scan match and scan-to-map match, Gaussian-Newton 
* need high rate scan data
* 2.5D SLAM
* multi-resolution map to avoid local minima
  
## 4.2 GMapping
* Rao-Blackwellized Particle filter approach

## 4.3 Karto SLAM
* graph-based approach
* landmark based
* Cholesky decomposition for solving sparse linar system
* loop closure
* sparse pose adjustment(just like bundle adjustment)

## 4.4 CoreSLAM
* ros wrapper of tinySLAM(200 lines-of-code)
* partical filter based approach
  
## 4.5 LagoSLAM
* Graph optimization -> Linear approximation of Graph optimization
* linear so no initial value needed

## 5. map comparision 
* grid_map https://github.com/ANYbotics/grid_map
* costmap_2d http://wiki.ros.org/costmap_2d
* OctoMap  https://octomap.github.io/

# 6. Reference
* An ICP variant using a point-to-line metric
* A flexible and Scalable SLAM System with Full 3D motion Estimation
* Real-Time Loop Closure in 2D LIDAR SLAM
* Grid-based Scan-to-Map Matching for Accurate 2D Map Building

