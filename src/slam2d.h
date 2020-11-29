#ifndef __SLAM2D_H
#define __SLAM2D_H
#include "slam2d_pose_graph.h"

#include <iostream>
#include <Eigen/Eigen>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/opencv.hpp>


using namespace std;
using namespace Eigen;
using namespace cv;


typedef pcl::PointXY PointType;

typedef struct
{
    double theta;
    Eigen::Vector2d t;

} state2d;

Eigen::Vector2d point2eigen(PointType p)
{
    Eigen::Vector2d pp;
    pp(0) = p.x;
    pp(1) = p.y;
    return pp;
}

PointType eigen2point(Eigen::Vector2d pp)
{
    PointType p;
    p.x = pp(0);
    p.y = pp(1);
    return p;
}

class slam2d
{
private:
    /* data */
public:
    slam2d(/* args */);
    ~slam2d();
    state2d state;
    state2d delta;
    double timestamp;
    nav_msgs::OccupancyGrid map2d;
    Mat cvmap2d;
    Mat cvmap2d_smooth;

    pcl::PointCloud<PointType> scan;
    pcl::PointCloud<PointType> scan_prev;

    pcl::PointCloud<PointType> scan_normal;
    bool cvmap_vis_enable = false;


    void readin_scan_data(const sensor_msgs::MultiEchoLaserScanConstPtr &msg);
    void readin_scan_data(const sensor_msgs::LaserScanConstPtr &msg);

    Vector2d world2map(Vector2d p);
    cv::Point2i world2map(cv::Point2f p);

    void update_scan_normal();
    void scan_match();
    void scan_map_match();
    void scan_map_match_random();
    void scan_map_match_bruce();
    VectorXd scan_map_match_cost(Vector3d pose);
    int scan_map_match_score(Vector3d pose);
    void update();
    void update_transform();

    void bresenham(Point2i p1, Point2i p2);
    void update_map();
    void cvmap2map();//convert cv map to map
};

slam2d::slam2d()
{
    state.t = Vector2d::Zero();
    state.theta = 0;
    map2d.header.frame_id = "odom";
    map2d.info.width = 2000;
    map2d.info.height = 2000;
    map2d.info.resolution = 0.1;
    map2d.info.origin.orientation.w = 1;
    map2d.info.origin.orientation.x = 0;
    map2d.info.origin.orientation.y = 0;
    map2d.info.origin.orientation.z = 0;
    map2d.info.origin.position.x = -0.5 * map2d.info.width * map2d.info.resolution;
    map2d.info.origin.position.y = -0.5 * map2d.info.height * map2d.info.resolution;
    map2d.info.origin.position.z = 0;
    map2d.data.resize(map2d.info.width * map2d.info.height);
    cvmap2d = Mat(map2d.info.width, map2d.info.height, CV_8SC1, -1);
    cvmap2d_smooth = Mat(map2d.info.width, map2d.info.height, CV_32FC1, 0.0f);
    cvmap2map();
}

slam2d::~slam2d()
{
}

void slam2d::readin_scan_data(const sensor_msgs::MultiEchoLaserScanConstPtr &msg)
{
    timestamp = msg->header.stamp.toSec();
    scan.points.resize(msg->ranges.size());
    for (auto i = 0; i < msg->ranges.size(); i++)
    {
        float dist = msg->ranges[i].echoes[0]; //only first echo used for slam2d
        float theta = msg->angle_min + i * msg->angle_increment;
        scan.points[i].x = dist * cos(theta);
        scan.points[i].y = dist * sin(theta);
    }
    scan.width = scan.points.size();
    scan.height = 1;
    scan.is_dense = true;
}
void slam2d::readin_scan_data(const sensor_msgs::LaserScanConstPtr &msg)
{
    timestamp = msg->header.stamp.toSec();
    scan.points.resize(msg->ranges.size());
    for (auto i = 0; i < msg->ranges.size(); i++)
    {
        float dist = msg->ranges[i]; //only first echo used for slam2d
        float theta = msg->angle_min + i * msg->angle_increment;
        scan.points[i].x = dist * cos(theta);
        scan.points[i].y = dist * sin(theta);
    }
    scan.width = scan.points.size();
    scan.height = 1;
    scan.is_dense = true;
}

cv::Point2i slam2d::world2map(cv::Point2f p)
{
    cv::Point2i m;
    m.x = roundf(p.x / map2d.info.resolution + map2d.info.width * 0.5);
    m.y = roundf(p.y / map2d.info.resolution + map2d.info.height * 0.5);
    return m;
}

Vector2d slam2d::world2map(Vector2d p)
{
    Vector2d m;
    m = p / map2d.info.resolution;
    m(0) += map2d.info.width * 0.5;
    m(1) += map2d.info.height * 0.5;
    return m;
}
void slam2d::update_scan_normal()
{
    //compute normal of scan
    scan_normal.points.resize(scan.points.size());
    for (auto i = 1; i < scan.points.size(); i++)
    {
        PointType p1 = scan.points[i - 1];
        PointType p2 = scan.points[i];

        Eigen::Vector2d delta(p1.x - p2.x, p1.y - p2.y);
        Eigen::Vector2d normal(-delta(1), delta(0));
        normal /= normal.norm();
        scan_normal.points[i].x = normal(0);
        scan_normal.points[i].y = normal(1);
    }
}
void slam2d::scan_match()
{
    double pose[3] = {0};
    if (scan.points.size() && scan_prev.points.size())
    {

        Problem problem;
        //solve delta with ceres constraints
        pcl::KdTreeFLANN<PointType> kdtree;
        kdtree.setInputCloud(scan.makeShared());
        int K = 2; // K nearest neighbor search
        std::vector<int> index(K);
        std::vector<float> distance(K);
        //1. project scan_prev to scan

        Eigen::Matrix2d R;
        R(0, 0) = cos(delta.theta); R(0, 1) = -sin(delta.theta);
        R(1, 0) = sin(delta.theta); R(1, 1) = cos(delta.theta);
        Eigen::Vector2d dt = delta.t;
        //find nearest neighur
        for (int i = 0; i < scan_prev.points.size(); i++)
        {
            PointType search_point = scan_prev.points[i];
            //project search_point to current frame
            PointType search_point_predict = eigen2point(R * point2eigen(search_point) + dt);
            if (kdtree.nearestKSearch(search_point_predict, K, index, distance) == K)
            {
                //add constraints
                Eigen::Vector2d p = point2eigen(search_point);
                Eigen::Vector2d p1 = point2eigen(scan.points[index[0]]);
                Eigen::Vector2d p2 = point2eigen(scan.points[index[1]]);
                ceres::CostFunction *cost_function = lidar_edge_error::Create(p, p1, p2);
                problem.AddResidualBlock(cost_function,
                                         new CauchyLoss(0.5),
                                         pose);
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

VectorXd slam2d::scan_map_match_cost(Vector3d pose)
{
    VectorXd residual = VectorXd::Zero(scan.points.size());
    Eigen::Matrix2d R;
    Vector2d t(pose(1), pose(2));
    double theta = pose(0);
    R(0, 0) = cos(theta); R(0, 1) = -sin(theta);
    R(1, 0) = sin(theta); R(1, 1) =  cos(theta);
    //printf("cols: %d, rows: %d\n", cvmap2d.cols, cvmap2d.rows);
    for (int i = 0; i < scan.points.size(); i++)
    {
        Vector2d p = point2eigen(scan.points[i]);
        Vector2d pp = world2map(R * p + t);
        //cout << "pp: " << pp.transpose() << endl;
        if ((pp(0) <= 1) || (pp(0) >= cvmap2d.cols) || (pp(1) <= 1) || (pp(1) >= cvmap2d.rows))
        {
            residual[i] = 0;
            //printf("res:%f\n", residual[i]);
        } else 
        {
            //get value from map
            double x = pp(0);
            double y = pp(1);
            int x2 = ceil(x);
            int x1 = x2 - 1;
            int y2 = ceil(y);
            int y1 = y2 - 1;
            float p11 = cvmap2d_smooth.at<float>(y1 * cvmap2d.cols + x1);
            float p12 = cvmap2d_smooth.at<float>(y1 * cvmap2d.cols + x2);
            float p21 = cvmap2d_smooth.at<float>(y2 * cvmap2d.cols + x1);
            float p22 = cvmap2d_smooth.at<float>(y2 * cvmap2d.cols + x2);
            float p1 = (x - x1) * p12 + (x2 - x) * p11;
            float p2 = (x - x1) * p22 + (x2 - x) * p21;
            float ppp = (y - y1) * p2 + (y2 - y) * p1;
            residual[i] = 100.0 - ppp;
            //printf("i: %d, res:%f\n", i, residual[i]);
        }
    }
    //generate local map and compute local optimal?
    return residual;
}

int slam2d::scan_map_match_score(Vector3d pose)
{
    int score = 0;
    Eigen::Matrix2d R;
    Vector2d t(pose(1), pose(2));
    double theta = pose(0);
    R(0, 0) = cos(theta); R(0, 1) = -sin(theta);
    R(1, 0) = sin(theta); R(1, 1) =  cos(theta);
    //printf("cols: %d, rows: %d\n", cvmap2d.cols, cvmap2d.rows);
    for (int i = 0; i < scan.points.size(); i++)
    {
        Vector2d p = point2eigen(scan.points[i]);
        Vector2d pp = world2map(R * p + t);
        //cout << "pp: " << pp.transpose() << endl;
        if ((pp(0) <= 1) || (pp(0) >= cvmap2d.cols) || (pp(1) <= 1) || (pp(1) >= cvmap2d.rows))
        {
            continue;
        } else 
        {
            //get value from map
            int x = round(pp(0));
            int y = round(pp(1));
            if (cvmap2d.at<int8_t>(y * cvmap2d.cols + x) == 100)
            {
                score++;
            }
            //printf("i: %d, res:%f\n", i, residual[i]);
        }
    }
    //generate local map and compute local optimal?
    return score;
}

void slam2d::scan_map_match()
{
    float eps = 1e-6;
    Vector3d pose(state.theta, state.t(0), state.t(1));

    VectorXd r = scan_map_match_cost(pose);
    MatrixXd H = MatrixXd::Zero(scan.points.size(), 3);
    for(int i = 0; i < 3; i++)
    {
        Vector3d pose1 = pose;
        pose1(i) += eps;
        VectorXd r1 = scan_map_match_cost(pose1);

        Vector3d pose2 = pose;
        pose2(i) -= eps;
        VectorXd r2 = scan_map_match_cost(pose2);

        H.col(i) = (r1 - r2 ) / (2 * eps);
    }
    cout << "H: " << H.transpose() << endl;
    cout << "r: " << r.transpose() << endl;
    Vector3d dx = (H.transpose() * H).ldlt().solve(H.transpose() * r);
    cout << "dx: " << dx.transpose() << endl;

    //update to state
    double mu = 0.5;
    state.theta += mu * dx(0);
    //state.t(0)  += mu * dx(1);
    //state.t(1)  += mu * dx(2);
}

void slam2d::scan_map_match_random()
{
    Vector3d pose(state.theta, state.t(0), state.t(1));
    double eps = 1e-5;
    //search best mattch
    int N = 200;

    for (int i = 0; i < N; i++)
    {
        //random direction
        Vector3d d = Vector3d::Random();
        d(0) /= 10.0;
        d.normalize();
        double min_len = 0;
        double max_len = 0.2;
        //search best len
        while((max_len - min_len) > eps)
        {
            int score1 = scan_map_match_score(pose + d * min_len);
            int score2 = scan_map_match_score(pose + d * max_len);
            if (score1 >= score2)
            {
                max_len = (min_len + max_len) / 2.0;
            } else 
            {
                min_len = (min_len + max_len) / 2.0;
            }
        }
        pose += d * min_len;
        Vector3d dx = d * min_len;
        int score = scan_map_match_score(pose);
        printf("score: %d, min_len: %lf\n", score, min_len);
        cout << "dx: " << dx.transpose() << endl;
    }
    //update to state
    state.theta = pose(0);
    state.t = pose.bottomRows(2);
}



void slam2d::scan_map_match_bruce()
{
}
void slam2d::update_transform()
{

    Eigen::Matrix2d dR;
    dR(0, 0) = cos(delta.theta); dR(0, 1) = -sin(delta.theta);
    dR(1, 0) = sin(delta.theta); dR(1, 1) = cos(delta.theta);


    Eigen::Vector2d dt_inv = -dR.transpose() * delta.t;
    Eigen::Matrix2d dR_inv = dR.transpose();
    

    Eigen::Matrix2d R;
    R(0, 0) = cos(state.theta); R(0, 1) = -sin(state.theta);
    R(1, 0) = sin(state.theta); R(1, 1) =  cos(state.theta);
    state.theta += (-delta.theta);
    state.t += R * dt_inv;
}

void slam2d::update()
{
    static int cnt = 0;
    if (scan.points.size() && scan_prev.points.size())
    {
        scan_match();
        update_transform();
        scan_map_match_random();
        update_map();
    }

    if (scan.points.size())
    {
        scan_prev = scan;
    }
    cnt++;
}

void slam2d::bresenham(Point2i p1, Point2i p2)
{
    //drawing a line from p1 to p2
    int dx = abs(p2.x - p1.x);
    int sx = (p2.x > p1.x) ? 1 : -1;
    int dy = abs(p2.y - p1.y);
    int sy = (p2.y > p1.y) ? 1 : -1;
    int err = (dx > dy ? dx : dy) / 2;
    int x1 = p1.x;
    int y1 = p1.y;
    int x2 = p2.x;
    int y2 = p2.y;

    while (x1 != x2 && y1 != y2)
    {
        if (cvmap2d.at<int8_t>(y1 * cvmap2d.cols + x1) == 100)
        {
            break;
        }
        else if (cvmap2d.at<int8_t>(y1 * cvmap2d.cols + x1) == -1)
        {
            cvmap2d.at<int8_t>(y1 * cvmap2d.cols + x1) = 0;
        }
        int e2 = err;
        if (e2 > -dx) 
        {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dy)
        {
            err += dx;
            y1 += sy;
        }
    }
}

void slam2d::update_map()
{
    //update map with scan and state
    cv::Point2f tt;
    tt.x = state.t(0);
    tt.y = state.t(1);
    cv::Point2i origin = world2map(tt);
    if (origin.x < 0 || origin.x >= cvmap2d.cols || origin.y < 0 || origin.y >= cvmap2d.rows) return;
    Eigen::Matrix2d R;
    R(0, 0) = cos(state.theta); R(0, 1) = -sin(state.theta);
    R(1, 0) = sin(state.theta); R(1, 1) =  cos(state.theta);
    for (int i = 0; i < scan.points.size(); i++)
    {
        PointType p = scan.points[i];
        float dist = sqrtf(p.x * p.x + p.y * p.y);
        if (dist > 20) continue;
        Eigen::Vector2d pp = R * point2eigen(p) + state.t;
        Point2f ppp(pp(0), pp(1));

        cv:Point2i pt = world2map(ppp);

        if (pt.x < 0 || pt.x >= cvmap2d.cols || pt.y < 0 || pt.y >= cvmap2d.rows)
            return;

        bresenham(origin, pt);
        //cv::line(cvmap2d, origin, pt, 0, 1, 4, 0);            //0->null
        cvmap2d.at<int8_t>(pt.y * cvmap2d.cols + pt.x) = 100; //100->occupancied
            //cv::circle(cvmap2d, ppp, 1, 0, 1, 4, 0);
    }
    cvmap2map();

    //blured image for scan-map align
    // Mat map;
    // cvmap2d.convertTo(map, CV_32FC1);
    // cv::GaussianBlur(map, cvmap2d_smooth, Size(5, 5), 0);
}

void slam2d::cvmap2map()
{
    for (int i = 0; i < cvmap2d.rows; i++)
    {
        for(int j = 0; j < cvmap2d.cols; j++)
        {
            map2d.data[i * map2d.info.width + j] = cvmap2d.at<int8_t>(i, j);
        }
    }
    if (cvmap_vis_enable)
    {
        imshow("cvmap2d", cvmap2d);
        imshow("cvmap2d_smooth", cvmap2d_smooth);
        waitKey(2);
    }
}
#endif
