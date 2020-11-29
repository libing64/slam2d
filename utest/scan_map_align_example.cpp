#include <iostream>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;
using namespace cv;

int width = 500;
int height = 500;

void map2map_align()
{

}

int main()
{
    //align and submap and map
    Mat map;
    Mat submap = Mat(width, height, CV_32FC1, -1);
    Point2f p1(0.1 * width, 0.1 * height);
    Point2f p2(0.9 * width, 0.1 * height);
    Point2f p3(0.9 * width, 0.9 * height);
    line(submap, p1, p2, 100);
    line(submap, p2, p3, 100);
    Mat rot = cv::getRotationMatrix2D(Point2f(submap.cols / 2, submap.rows / 2), 5.0, 1.0);
    warpAffine(submap, map, rot, submap.size());
    imshow("submap", submap);
    imshow("map", map);

    waitKey();
    return 0;
}