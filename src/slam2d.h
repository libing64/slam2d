#ifndef __SLAM2D_H
#define __SLAM2D_H
#include <iostream>

#include <Eigen/Eigen>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MultiEchoLaserScan.h>

#include "slam2d_pose_graph.h"
#include "voxel_map.h"

using namespace std;
using namespace Eigen;
using namespace cv;

typedef pcl::PointXY PointType;

typedef struct {
  double theta;
  Eigen::Vector2d t;

} state2d;

Eigen::Vector2d point2eigen(PointType p) {
  Eigen::Vector2d pp;
  pp(0) = p.x;
  pp(1) = p.y;
  return pp;
}

PointType eigen2point(Eigen::Vector2d pp) {
  PointType p;
  p.x = pp(0);
  p.y = pp(1);
  return p;
}

class slam2d {
 private:
  /* data */
 public:
  slam2d(/* args */);
  ~slam2d();
  state2d state;
  state2d delta;
  state2d last_keyframe_state;
  double timestamp;
  nav_msgs::OccupancyGrid map2d;
  VoxelMap2D voxel_map;

  pcl::PointCloud<PointType> scan;
  pcl::PointCloud<PointType> scan_prev;

  bool cvmap_vis_enable = false;

  void readin_scan_data(const sensor_msgs::MultiEchoLaserScanConstPtr& msg);
  void readin_scan_data(const sensor_msgs::LaserScanConstPtr& msg);

  Vector2d world2map(Vector2d p);
  cv::Point2i world2map(cv::Point2f p);

  void scan_match();
  void scan_map_match_b2b();
  void scan_map_match_random();
  float scan_map_match_score(Vector3d pose);
  void update();
  void update_transform();

  void update_map();
  void voxel2rosmap();  // convert voxel map to ros map
};

slam2d::slam2d() : voxel_map(2000, 2000, 0.15) {
  state.t = Vector2d::Zero();
  state.theta = 0;
  last_keyframe_state.t = Vector2d::Zero();
  last_keyframe_state.theta = 0;
  map2d.header.frame_id = "odom";
  map2d.info.width = voxel_map.width;
  map2d.info.height = voxel_map.height;
  map2d.info.resolution = voxel_map.resolution;
  map2d.info.origin.orientation.w = 1;
  map2d.info.origin.orientation.x = 0;
  map2d.info.origin.orientation.y = 0;
  map2d.info.origin.orientation.z = 0;
  map2d.info.origin.position.x = voxel_map.origin_offset.x();
  map2d.info.origin.position.y = voxel_map.origin_offset.y();
  map2d.info.origin.position.z = 0;
  map2d.data.resize(map2d.info.width * map2d.info.height, -1);
  voxel2rosmap();
}

slam2d::~slam2d() {}

void slam2d::readin_scan_data(
    const sensor_msgs::MultiEchoLaserScanConstPtr& msg) {
  timestamp = msg->header.stamp.toSec();
  scan.points.resize(msg->ranges.size());
  for (size_t i = 0; i < msg->ranges.size(); i++) {
    float dist = msg->ranges[i].echoes[0];  // only first echo used for slam2d
    float theta = msg->angle_min + i * msg->angle_increment;
    scan.points[i].x = dist * cos(theta);
    scan.points[i].y = dist * sin(theta);
  }
  scan.width = scan.points.size();
  scan.height = 1;
  scan.is_dense = true;
}
void slam2d::readin_scan_data(const sensor_msgs::LaserScanConstPtr& msg) {
  timestamp = msg->header.stamp.toSec();
  scan.points.resize(msg->ranges.size());
  for (size_t i = 0; i < msg->ranges.size(); i++) {
    float dist = msg->ranges[i];  // only first echo used for slam2d
    float theta = msg->angle_min + i * msg->angle_increment;
    scan.points[i].x = dist * cos(theta);
    scan.points[i].y = dist * sin(theta);
  }
  scan.width = scan.points.size();
  scan.height = 1;
  scan.is_dense = true;
}

cv::Point2i slam2d::world2map(cv::Point2f p) {
  cv::Point2i m;
  m.x = roundf(p.x / map2d.info.resolution + map2d.info.width * 0.5);
  m.y = roundf(p.y / map2d.info.resolution + map2d.info.height * 0.5);
  return m;
}

Vector2d slam2d::world2map(Vector2d p) {
  Vector2d m;
  m = p / map2d.info.resolution;
  m(0) += map2d.info.width * 0.5;
  m(1) += map2d.info.height * 0.5;
  return m;
}

void slam2d::scan_match() {
  double pose[3] = {0};
  if (scan.points.size() && scan_prev.points.size()) {
    Problem problem;
    // solve delta with ceres constraints
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(scan.makeShared());
    int K = 2;  // K nearest neighbor search
    std::vector<int> index(K);
    std::vector<float> distance(K);
    // 1. project scan_prev to scan

    Eigen::Matrix2d R;
    R(0, 0) = cos(delta.theta);
    R(0, 1) = -sin(delta.theta);
    R(1, 0) = sin(delta.theta);
    R(1, 1) = cos(delta.theta);
    Eigen::Vector2d dt = delta.t;
    // find nearest neighur
    for (int i = 0; i < scan_prev.points.size(); i++) {
      PointType search_point = scan_prev.points[i];
      // project search_point to current frame
      PointType search_point_predict =
          eigen2point(R * point2eigen(search_point) + dt);
      if (kdtree.nearestKSearch(search_point_predict, K, index, distance) ==
          K) {
        // add constraints
        Eigen::Vector2d p = point2eigen(search_point);
        Eigen::Vector2d p1 = point2eigen(scan.points[index[0]]);
        Eigen::Vector2d p2 = point2eigen(scan.points[index[1]]);
        ceres::CostFunction* cost_function =
            lidar_edge_error::Create(p, p1, p2);
        problem.AddResidualBlock(cost_function, new CauchyLoss(0.5), pose);
      }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    printf("result: %lf, %lf, %lf\n", pose[0], pose[1], pose[2]);

    delta.theta = pose[0];
    delta.t(0) = pose[1];
    delta.t(1) = pose[2];
  }
}

float slam2d::scan_map_match_score(Vector3d pose) {
  float score = 0;
  Eigen::Matrix2d R;
  Vector2d t(pose(1), pose(2));
  double theta = pose(0);
  R(0, 0) = cos(theta);
  R(0, 1) = -sin(theta);
  R(1, 0) = sin(theta);
  R(1, 1) = cos(theta);

  for (int i = 0; i < scan.points.size(); i++) {
    Vector2d p = point2eigen(scan.points[i]);
    Vector2d pp = R * p + t;

    // Use bilinear interpolation for sub-pixel precision mapping
    score += voxel_map.getProbBilinear(pp);
  }
  return score;
}

void slam2d::scan_map_match_b2b() {
  BranchAndBoundMatcher b2b_matcher(voxel_map.resolution,
                                    0.05);  // 0.05 rad ~ 2.8 deg angular res
  Vector3d initial_pose(state.theta, state.t(0), state.t(1));
  Vector3d best_pose = b2b_matcher.match(voxel_map, scan, initial_pose);

  // update to state
  state.theta = best_pose(0);
  state.t = best_pose.bottomRows(2);
}

void slam2d::scan_map_match_random() {
  Vector3d pose(state.theta, state.t(0), state.t(1));
  double eps = 1e-5;
  // search best mattch
  int N = 200;

  for (int i = 0; i < N; i++) {
    // random direction
    Vector3d d = Vector3d::Random();
    d(0) /= 10.0;
    d.normalize();
    double min_len = 0;
    double max_len = 0.2;
    // search best len
    while ((max_len - min_len) > eps) {
      int score1 = scan_map_match_score(pose + d * min_len);
      int score2 = scan_map_match_score(pose + d * max_len);
      if (score1 >= score2) {
        max_len = (min_len + max_len) / 2.0;
      } else {
        min_len = (min_len + max_len) / 2.0;
      }
    }
    pose += d * min_len;
    Vector3d dx = d * min_len;
    int score = scan_map_match_score(pose);
    printf("score: %d, min_len: %lf\n", score, min_len);
    cout << "dx: " << dx.transpose() << endl;
  }
  // update to state
  state.theta = pose(0);
  state.t = pose.bottomRows(2);
}

void slam2d::update_transform() {
  Eigen::Matrix2d dR;
  dR(0, 0) = cos(delta.theta);
  dR(0, 1) = -sin(delta.theta);
  dR(1, 0) = sin(delta.theta);
  dR(1, 1) = cos(delta.theta);

  Eigen::Vector2d dt_inv = -dR.transpose() * delta.t;
  Eigen::Matrix2d dR_inv = dR.transpose();

  Eigen::Matrix2d R;
  R(0, 0) = cos(state.theta);
  R(0, 1) = -sin(state.theta);
  R(1, 0) = sin(state.theta);
  R(1, 1) = cos(state.theta);
  state.theta += (-delta.theta);
  state.t += R * dt_inv;
}

void slam2d::update() {
  static int cnt = 0;
  if (scan.points.size() && scan_prev.points.size()) {
    scan_match();
    update_transform();

    // Keyframe selection to reduce drift
    double dx = state.t(0) - last_keyframe_state.t(0);
    double dy = state.t(1) - last_keyframe_state.t(1);
    double dtheta = state.theta - last_keyframe_state.theta;

    // Normalize dtheta to [-pi, pi]
    while (dtheta > M_PI) dtheta -= 2 * M_PI;
    while (dtheta < -M_PI) dtheta += 2 * M_PI;

    double dist = sqrt(dx * dx + dy * dy);

    if (dist > 2.0 || fabs(dtheta) > 0.3) {
      scan_map_match_b2b();
      update_map();
      last_keyframe_state = state;
    }
  } else if (scan.points.size() && !scan_prev.points.size()) {
    // First frame initialization
    last_keyframe_state = state;
    update_map();
  }

  if (scan.points.size()) {
    scan_prev = scan;
  }
  cnt++;
}

void slam2d::update_map() {
  // update map with scan and state
  Vector2d origin = state.t;
  Vector2i origin_idx = voxel_map.worldToMap(origin);

  if (!voxel_map.isInside(origin_idx.x(), origin_idx.y())) return;

  Eigen::Matrix2d R;
  R(0, 0) = cos(state.theta);
  R(0, 1) = -sin(state.theta);
  R(1, 0) = sin(state.theta);
  R(1, 1) = cos(state.theta);

  for (size_t i = 0; i < scan.points.size(); i++) {
    PointType p = scan.points[i];
    float dist = sqrtf(p.x * p.x + p.y * p.y);
    if (dist > 20) continue;
    Eigen::Vector2d pp = R * point2eigen(p) + state.t;

    Vector2i pt_idx = voxel_map.worldToMap(pp);

    if (!voxel_map.isInside(pt_idx.x(), pt_idx.y())) continue;

    voxel_map.bresenham(origin_idx, pt_idx);
  }
  voxel2rosmap();
}

void slam2d::voxel2rosmap() {
  for (int y = 0; y < voxel_map.height; y++) {
    for (int x = 0; x < voxel_map.width; x++) {
      float prob = voxel_map.getProb(x, y);
      int8_t val = -1;  // unknown
      if (prob > 0.65)
        val = 100;  // occupied
      else if (prob < 0.45)
        val = 0;  // free

      map2d.data[y * map2d.info.width + x] = val;
    }
  }

  if (cvmap_vis_enable) {
    cv::Mat map_img(voxel_map.height, voxel_map.width, CV_8UC1);
    for (int y = 0; y < voxel_map.height; y++) {
      for (int x = 0; x < voxel_map.width; x++) {
        int8_t v = map2d.data[y * map2d.info.width + x];
        if (v == -1)
          map_img.at<uint8_t>(y, x) = 127;
        else if (v == 100)
          map_img.at<uint8_t>(y, x) = 0;
        else
          map_img.at<uint8_t>(y, x) = 255;
      }
    }
    cv::imshow("cvmap2d", map_img);
    cv::waitKey(2);
  }
}
#endif
