//
// Created by finn on 27.05.21.
//

#ifndef SRC_RRT_H
#define SRC_RRT_H
#include "rrt_node.h"
#include <tuple>
#include <flann/algorithms/dist.h>
#include <flann/algorithms/kdtree_single_index.h>
#include <flann/flann.hpp>
typedef  vector<vector<vector<vector<vector<vector<vector<rrt_node*>>>>>>> node_grid;

class rrt {
private:

    // Flann
    flann::Index<flann::L2_Simple<float>> kdtree = flann::Index<flann::L2_Simple<float>>(
            flann::KDTreeSingleIndexParams());
    void point_to_flann(Point p, float* data){
        data[0] = p[0];
        data[1] = p[1];
        data[2] = p[2];
        data[3] = p[3];
        data[4] = p[4];
        data[5] = p[5];
    }
    Point flann_to_point(float* data){
        return (Point){data[0],data[1],data[2],data[3],data[4],data[5]};
    }
    std::unordered_map<Point, rrt_node*> nodemap;
    Point goal_point;
    Point start_point;
    rrt_node* start_node;
    rrt_node* goal_node = nullptr;
    // RRT Params
    rrt_params params;
    // TODO save obstacles
    rrt_node* findNearestNode(Point relative_to);
    normal_random_variable* sampler;
    // Sorting grid
    node_grid grid;
    array<int, 6> num_cells;
    array<int, 6> return_grid_id(Point point);
public:
    long num_nodes = 0;
    rrt(Point start_point, Point goal_point,rrt_params params);
    tuple<bool, array<Point, 2>> expand();
    vector<Point> return_goal_path();

};

#endif  // SRC_RRT_H
