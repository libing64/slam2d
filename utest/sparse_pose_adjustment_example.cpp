#include <Eigen/Eigen>
#include <ceres/ceres.h>
#include <iostream>
#include "spa_pose_graph.h"

using namespace Eigen;
using namespace std;





typedef struct
{
    int i;
    int j;
    Vector3d delta;
} lidar2d_meas_t;


int main()
{
    const int N = 1000;
    const int M = 10;
    //1. generate state
    MatrixXd inc = MatrixXd::Random(3, N) * 0.1;
    MatrixXd state = MatrixXd::Zero(3, N);
    for (int i = 1; i < N; i++)
    {
        state.col(i) = state.col(i - 1) + inc.col(i);
    }
    vector<lidar2d_meas_t> meas_q;
    //2. generate measurement of scan-scan match
    for (int i = 0; i < N; i++)
    {
        for (int j = i + 1; j <= i + M; j++)
        {
            if (j < N) 
            {
                lidar2d_meas_t meas;
                meas.i = i;
                meas.j = j;

                Matrix2d R = Matrix2d::Zero();
                double theta = state(0, i);
                R(0, 0) = cos(theta); R(0, 1) = -sin(theta);
                R(1, 0) = sin(theta); R(1, 1) = cos(theta);
                meas.delta(0) = state(0, j) - state(0, i);
                meas.delta.bottomRows(2) = R.transpose() * (state.col(j).bottomRows(2) - state.col(i).bottomRows(2));
                meas.delta += Vector3d::Random() * 0.01;//add noise
                meas_q.push_back(meas);
            }
        }
    }
    //3. solve sparse bundle adjustment equation
    Problem problem;
    int NN = meas_q.size();
    MatrixXd H = MatrixXd::Zero(NN * 3, N * 3);
    double *pose = new double[N * 3];
    for (int i = 0; i < N * 3; i++)
    {
        pose[i] = 0;
    }
    for (int k = 0; k < NN; k++)
    {
        lidar2d_meas_t meas = meas_q[k];
        int i = meas.i;
        int j = meas.j;
        ceres::CostFunction *cost_function = spa_edge_error::Create(meas.delta);
        problem.AddResidualBlock(cost_function,
                                 NULL,//new CauchyLoss(0.5),
                                 &pose[i * 3], &pose[j * 3]);
    }
    //constraints for first pose
    ceres::CostFunction *cost_function = spa_origin_error::Create(Vector3d::Zero());
    problem.AddResidualBlock(cost_function,
                             NULL, //new CauchyLoss(0.5),
                             pose);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    for (int i = 0; i < N; i += 10)
    {
        printf("i: %d, %lf, %lf, %lf\n", i, pose[i * 3 + 0], pose[i * 3 + 1], pose[i * 3 + 2]);
        printf("sta: %lf, %lf, %lf\n", state(0, i), state(1, i), state(2, i));
        printf("err: %lf, %lf, %lf\n", state(0, i) - pose[i * 3 + 0], state(1, i) - pose[i * 3 + 1], state(2, i) - pose[i * 3 + 2]);
    }
    
    return 0;
}