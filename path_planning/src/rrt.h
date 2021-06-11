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
#include <boost/math/interpolators/cubic_b_spline.hpp>
#include <eigen3/Eigen/SVD>
typedef vector<vector<vector<vector<rrt_node*>>>> node_grid;
using namespace Eigen;
class rrt {
private:
    // informed rrt
    float c_opt;
    Matrix<float, 6, 1> center;
    Matrix<float, 6, 6> C;
    Matrix<float, 6, 6> L;
    // kinematics
    Matrix<float, 8, 1> a;
    Matrix<float, 8, 1> d;
    Matrix<float, 8, 1> alpha;
    // Flann
    flann::Index<flann::L2_Simple<float>> kdtree;
    vector<rrt_node*> all_nodes;
    joint_angles goal_point;
    joint_angles goal_point_found;
    bool goal_found = false;
    Point goal_p;
    joint_angles start_point;
    rrt_node* start_node;
    Point goal_normal = {0, 0, -1};
    // RRT Params
    rrt_params params;
    // TODO save obstacles
    rrt_node* findNearestNode(joint_angles& relative_to);
    vector<rrt_node*> findNearNodes(joint_angles& relative_to);
    Point get_end_effector(joint_angles angles);
    Point get_end_effector_normal(joint_angles angles);
    joint_angles sample_ellipsoid();
    void calculateC(joint_angles gp);
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    long long num_nodes = 0;
    long long nodesmark_goal_found = 0;
    float min_dist = -1;
    float min_dist_orient = -1;
    rrt_node* goal_node = nullptr;
    rrt(joint_angles start_point, joint_angles goal_point,rrt_params params);
    tuple<bool, array<Point, 2>> expand();
    vector<tuple<Point, joint_angles>> return_goal_path();

};

#endif  // SRC_RRT_H
