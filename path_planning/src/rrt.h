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
typedef vector<vector<vector<vector<rrt_node*>>>> node_grid;
using namespace Eigen;
class rrt {
private:
    Matrix<float, 8, 1> a;
    Matrix<float, 8, 1> d;
    Matrix<float, 8, 1> alpha;
    // Flann
    flann::Index<flann::L2_Simple<float>> kdtree;
    vector<rrt_node*> all_nodes;
    std::unordered_map<Point, rrt_node*, std::function<size_t(Point)>> nodemap;
    Point goal_point;
    array<float, 3> goal_p;
    Point start_point;
    rrt_node* start_node;
    rrt_node* goal_node = nullptr;
    // RRT Params
    rrt_params params;
    // TODO save obstacles
    rrt_node* findNearestNode(Point& relative_to);
    normal_random_variable* sampler;

    Point get_end_effector(Point angles);
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    long num_nodes = 0;
    rrt(Point start_point, Point goal_point,rrt_params params);
    tuple<bool, array<Point, 2>> expand();
    vector<Point> return_goal_path();

};

#endif  // SRC_RRT_H
