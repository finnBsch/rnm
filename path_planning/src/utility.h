//
// Created by finn on 27.05.21.
//

#ifndef SRC_UTILITY_H
#define SRC_UTILITY_H
#include <array>
#include <vector>
#include <list>
#include <cmath>
#include "eigen3/Eigen/Dense"
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <random>
#include <boost/array.hpp>

using namespace std;
using namespace Eigen;
typedef boost::array<float, 3> Point;
typedef boost::array<float, 6> joint_angles;
typedef boost::array<float, 7> joint_angles_full;

struct rrt_params{
    float step_size;
    array<array<float, 2>, 6> joint_ranges;
    bool goal_joint;
    int num_nodes_min;
    float steercost;
};
void point_to_flann(joint_angles p, float* data);
joint_angles flann_to_point(float* data);
struct normal_random_variable
{
    normal_random_variable(Eigen::MatrixXd const& covar)
            : normal_random_variable(Eigen::VectorXd::Zero(covar.rows()), covar)
    {}

    normal_random_variable(Eigen::VectorXd const& mean, Eigen::MatrixXd const& covar)
            : mean(mean)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
        transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
    }

    Eigen::VectorXd mean;
    Eigen::MatrixXd transform;

    Eigen::VectorXd operator()() const
    {
        static std::mt19937 gen{ std::random_device{}() };
        static std::normal_distribution<> dist;

        return mean + transform * Eigen::VectorXd{ mean.size() }.unaryExpr([&](auto x) { return dist(gen); });
    }
};
float euclidean_dist(Point A, Point B);
float euclidean_dist_sqrd(Point A, Point B);
float euclidean_dist_sqrd_joint(joint_angles A, joint_angles B);
float euclidean_dist_joint(joint_angles A, joint_angles B);
float euclidean_norm(joint_angles A);
joint_angles random_point(array<array<float, 2>, 6> joint_ranges);
joint_angles step_forward(joint_angles start, joint_angles goal, float dist);



#endif //SRC_UTILITY_H
