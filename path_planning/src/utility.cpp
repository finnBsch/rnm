//
// Created by finn on 27.05.21.
//

#include "utility.h"
float euclidean_dist(Point A, Point B){
    float dx =A[0]-B[0];
    float dy = A[1]-B[1];
    float dz = A[2]-B[2];
    return sqrt((dx)*(dx) + (dy)*(dy) + (dz)*(dz));
}
float euclidean_dist_sqrd(Point A, Point B){
    float dx =A[0]-B[0];
    float dy = A[1]-B[1];
    float dz = A[2]-B[2];
    return ((dx)*(dx) + (dy)*(dy) + (dz)*(dz));
}
float euclidean_norm(Point A){
    return sqrt((A[0])*(A[0]) + (A[1])*(A[1]) + (A[2])*(A[2]));
}

Point random_point(array<float, 2> range_x, array<float, 2> range_y, array<float, 2> range_z){
    // TODO Random sampling
    // TODO Sample in sphere (workspace)
    float LO = range_x[0];
    float HI = range_x[1];
    float x_return = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
    LO = range_y[0];
    HI = range_y[1];
    float y_return = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
    LO = range_z[0];
    HI = range_z[1];
    float z_return = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
    return (Point){x_return, y_return, z_return};
}

Point step_forward(Point start, Point goal, float dist){
    Point diff = {goal[0]-start[0], goal[1]-start[1], goal[2]-start[2]};
    float norm = euclidean_norm(diff);
    return (Point){start[0] + diff[0]*dist/norm, start[1] + diff[1]*dist/norm, start[2] + diff[2]*dist/norm};
}