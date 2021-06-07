//
// Created by finn on 27.05.21.
//

#ifndef SRC_RRT_H
#define SRC_RRT_H
#include "rrt_node.h"
#include <tuple>
#include "ros/ros.h"
#include <flann/algorithms/dist.h>
#include <flann/algorithms/kdtree_single_index.h>
#include <flann/flann.hpp>
#include <eigen3/Eigen/Dense>
#include "forward_kin/get_endeffector.h"
typedef vector<vector<vector<vector<rrt_node*>>>> node_grid;
using namespace Eigen;
class rrt {
private:
    Matrix<double, 8, 1> a;
    Matrix<double, 8, 1> d;
    Matrix<double, 8, 1> alpha;
    // Flann
    flann::Index<flann::L2_Simple<double>> kdtree;
    vector<rrt_node*> all_nodes;
    Point goal_point;
    Point start_point;
    rrt_node* start_node;
    rrt_node* goal_node = nullptr;
    // RRT Params
    rrt_params params;
    // TODO save obstacles
    rrt_node* findNearestNode(Point& relative_to);
    normal_random_variable* sampler;
    forward_kin::get_endeffector srv;
    joint_angles generateSuccessor(rrt_node* root, Point goal_point);
    Point get_end_effector(joint_angles angles);
    //MatrixXd get_transformationmatrix(const float theta, const float a, const float d, const float alpha);


public:
    long num_nodes = 0;
    rrt(Point start_point, Point goal_point,rrt_params params, joint_angles init);
    tuple<bool, array<Point, 2>> expand();
    vector<tuple<Point, joint_angles>> return_goal_path();

};

#endif  // SRC_RRT_H
