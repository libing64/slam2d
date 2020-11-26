#include "slam2d.h"

#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <ros/ros.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Eigen>

slam2d slam;

void multiecho_laserscan_callback(const sensor_msgs::MultiEchoLaserScanConstPtr &msg)
{
    slam.update(msg);
}

void laserscan_callback(const sensor_msgs::LaserScanConstPtr &msg)
{
    slam.update(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam2d");
    ros::NodeHandle nh;

    ros::Subscriber sub_multiecho_laserscan = nh.subscribe<sensor_msgs::MultiEchoLaserScan>("/multiecho_scan", 100, multiecho_laserscan_callback);
    ros::Subscriber sub_laserscan = nh.subscribe<sensor_msgs::LaserScan>("/scan", 100, laserscan_callback);
    ros::spin();
    return 0;
}