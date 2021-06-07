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
using namespace std;
typedef array<float, 3> Point;
struct rrt_params{
    float step_size;
    array<array<float, 2>, 3> ranges;
    // TODO: find way to choose workspace
};
void point_to_flann(Point p, float* data);
Point flann_to_point(float* data);
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
float euclidean_norm(Point A);
Point random_point(array<array<float, 2>, 3> joint_ranges);
Point step_forward(Point start, Point goal, float dist);




#endif //SRC_UTILITY_H
