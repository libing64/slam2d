#ifndef _LIDAR_POSE_GRAPH_H
#define _LIDAR_POSE_GRAPH_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/dynamic_autodiff_cost_function.h>
#include <Eigen/Eigen>

using ceres::AngleAxisRotatePoint;
using ceres::AutoDiffCostFunction;
using ceres::CauchyLoss;
using ceres::CostFunction;
using ceres::CrossProduct;
using ceres::DotProduct;
using ceres::DynamicAutoDiffCostFunction;
using ceres::LossFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

//parameters: rotation and translation
//rotation in angleAxis format

struct spa_edge_error
{
    spa_edge_error(const Eigen::Vector3d delta)
        : delta(delta) {}

    template <typename T>
    bool operator()(const T *const p1, const T* const p2,
                    T *residuals) const
    {
        // pose[0] = theta, poes[1,2] = t
        Eigen::Matrix<T, 2, 2> Rt;
        Eigen::Matrix<T, 2, 1> t1, t2;

        T theta = p1[0];
        Rt(0, 0) =  cos(theta);
        Rt(0, 1) =  sin(theta);
        Rt(1, 0) = -sin(theta);
        Rt(1, 1) =  cos(theta);
        t1(0) = p1[1];
        t1(1) = p1[2];

        t2(0) = p2[1];
        t2(1) = p2[2];

        Eigen::Matrix<T, 3, 1> delta_hat;
        delta_hat(0) = p2[0] - p1[0];
        delta_hat.bottomRows(2) = Rt * (t2 - t1);
        residuals[0] = delta(0) - delta_hat(0);
        residuals[1] = delta(1) - delta_hat(1);
        residuals[2] = delta(2) - delta_hat(2);
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction *Create(const Eigen::Vector3d delta)
    {
        return (new ceres::AutoDiffCostFunction<spa_edge_error, 3, 3, 3>(
            new spa_edge_error(delta)));
    }

    //project point p to line (p1 - p2)
    Eigen::Vector3d delta;
};

struct spa_origin_error
{
    spa_origin_error(const Eigen::Vector3d delta)
        : delta(delta) {}

    template <typename T>
    bool operator()(const T *const p,
                    T *residuals) const
    {
        T weight = T(100.0);
        residuals[0] = weight * (delta(0) - p[0]);
        residuals[1] = weight * (delta(1) - p[1]);
        residuals[2] = weight * (delta(2) - p[2]);
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction *Create(const Eigen::Vector3d delta)
    {
        return (new ceres::AutoDiffCostFunction<spa_origin_error, 3, 3>(
            new spa_origin_error(delta)));
    }

    //project point p to line (p1 - p2)
    Eigen::Vector3d delta;
};

#endif