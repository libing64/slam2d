#ifndef _LIDAR_POSE_GRAPH_H
#define _LIDAR_POSE_GRAPH_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/dynamic_autodiff_cost_function.h>
#include <Eigen/Eigen>

using ceres::AutoDiffCostFunction;
using ceres::CauchyLoss;
using ceres::CostFunction;
using ceres::DynamicAutoDiffCostFunction;
using ceres::LossFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using ceres::AngleAxisRotatePoint;
using ceres::CrossProduct;
using ceres::DotProduct;

//parameters: rotation and translation
//rotation in angleAxis format

struct lidar_edge_error
{
    lidar_edge_error(const Eigen::Vector2d p, const Eigen::Vector2d p1, const Eigen::Vector2d p2)
        : p(p), p1(p1), p2(p2) {}

    template <typename T>
    bool operator()(const T *const pose,
                    T *residuals) const
    {
        // pose[0] = theta
        T pi[2];
        pi[0] = T(p(0));
        pi[1] = T(p(1));

        T R[2][2];
        R[0][0] = cos(pose[0]); R[0][1] = -sin(pose[0]); 
        R[1][0] = sin(pose[0]); R[1][1] =  cos(pose[0]); 
        T pi_proj[2];//project pi to current frame

        pi_proj[0] = R[0][0] * pi[0] + R[0][1] * pi[1];
        pi_proj[1] = R[1][0] * pi[0] + R[1][1] * pi[1];
        // pose[1, 2] are the translation.
        pi_proj[0] += pose[1];
        pi_proj[1] += pose[2];

        //distance between pi_proj to line(p1, p2)
        T d1[2], d12[2];
        d1[0] = pi_proj[0] - T(p1(0));
        d1[1] = pi_proj[1] - T(p1(1));

        d12[0] = T(p1(0) - p2(0));
        d12[1] = T(p1(1) - p2(1));

        T normal[2];
        normal[0] = -d12[1];
        normal[1] = d12[0];

        T norm = sqrt(normal[0] * normal[0] + normal[1] * normal[1]);
        normal[0] /= norm;
        normal[1] /= norm;

        residuals[0] = d1[0] * normal[0] + d1[1] * normal[1];//dot product
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction *Create(const Eigen::Vector2d p, const Eigen::Vector2d p1, const Eigen::Vector2d p2)
    {
        return (new ceres::AutoDiffCostFunction<lidar_edge_error, 1, 6>(
            new lidar_edge_error(p, p1, p2)));
    }


    //project point p to line (p1 - p2)
    Eigen::Vector2d p;
    Eigen::Vector2d p1;
    Eigen::Vector2d p2;
};



#endif