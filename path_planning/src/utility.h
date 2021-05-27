//
// Created by finn on 27.05.21.
//

#ifndef SRC_UTILITY_H
#define SRC_UTILITY_H
#include <array>
#include <vector>
#include <list>
#include <cmath>

using namespace std;
typedef array<float, 3> Point;
struct rrt_params{
    float step_size;
    array<float, 2> x_range;
    array<float, 2> y_range;
    array<float, 2> z_range;
    // TODO: find way to choose workspace
    float grid_size;
};
float euclidean_dist(Point A, Point B);
float euclidean_norm(Point A);
Point random_point(array<float, 2> range_x, array<float, 2> range_y, array<float, 2> range_z);
Point step_forward(Point start, Point goal, float dist);




#endif //SRC_UTILITY_H
