#ifndef __SLAM2D_H
#define __SLAM2D_H

#include <iostream>
#include <Eigen/Eigen>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;
using namespace Eigen;

class slam2d
{
private:
    /* data */
public:
    slam2d(/* args */);
    ~slam2d();

    vector<float> scan;
    void update(const sensor_msgs::MultiEchoLaserScanConstPtr& msg);
    void update(const sensor_msgs::LaserScanConstPtr &msg);
};

slam2d::slam2d(/* args */)
{
}

slam2d::~slam2d()
{
}

void slam2d::update(const sensor_msgs::MultiEchoLaserScanConstPtr &msg)
{
    printf("\n\nscan received:\n");
    scan.resize(msg->ranges.size());
    for (auto i = 0; i < msg->ranges.size(); i++)
    {
        scan[i] = msg->ranges[i].echoes[0];//only first echo used for slam2d
        printf("%f,", scan[i]);
    }
}

void slam2d::update(const sensor_msgs::LaserScanConstPtr &msg)
{

}
#endif
