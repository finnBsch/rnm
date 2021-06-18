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
#include "ecl/geometry.hpp"
#include "spline.h"
#include <ecl/geometry/tension_spline.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <random>
#include <boost/array.hpp>

using namespace std;
using namespace Eigen;
typedef boost::array<double, 3> Point;
typedef boost::array<double, 6> joint_angles;
typedef boost::array<double, 7> joint_angles_full;
struct rrt_params{
    double step_size;
    array<array<double, 2>, 6> joint_ranges;
    bool goal_joint;
    int num_nodes_extra;
    std::vector<double> max_vels;
    std::vector<double> max_accs;

};
void point_to_flann(joint_angles p, double* data);
joint_angles flann_to_point(double* data);
Matrix<double, 6, 1> sample_unit_ball();
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
double euclidean_dist(Point A, Point B);
double euclidean_dist_sqrd(Point A, Point B);
double euclidean_dist_sqrd_joint(joint_angles A, joint_angles B);
double euclidean_dist_joint(joint_angles A, joint_angles B);
double euclidean_norm(joint_angles A);
joint_angles random_point(array<array<double, 2>, 6> joint_ranges);
joint_angles step_forward(joint_angles start, joint_angles goal, double dist);



#endif //SRC_UTILITY_H
