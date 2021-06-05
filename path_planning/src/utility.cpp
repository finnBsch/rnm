//
// Created by finn on 27.05.21.
//

#include "utility.h"
float euclidean_dist(Point A, Point B){
    float distances;
    float total_distance = 0;
    for(int i = 0; i < A.size(); i++){
        distances = A[i] - B[i];
        total_distance += distances*distances;
    }
    return sqrt(total_distance);
}
float euclidean_dist_sqrd(Point A, Point B){
    float distances;
    float total_distance = 0;
    for(int i = 0; i < A.size(); i++){
        distances = A[i] - B[i];
        total_distance += distances*distances;
    }
    return total_distance;
}
float euclidean_norm(Point A){
    float distances;
    float total_distance = 0;
    for(int i = 0; i < A.size(); i++){
        distances = A[i];
        total_distance += distances*distances;
    }
    return sqrt(total_distance);
}

Point random_point(array<array<float, 2>, 6> joint_ranges){
    Point sampled;
    float LO;
    float HI;
    float sampled_angle;
    int i = 0;
    for(array<float, 2> range:joint_ranges){
        LO = range[0];
        HI = range[1];
        sampled_angle = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
        sampled[i] = sampled_angle;
        i++;
    }
    return sampled;
}

Point step_forward(Point start, Point goal, float dist){
    Point diff;
    for(int i = 0; i<start.size(); i++){
        diff[i] = goal[i]-start[i];
    }
    float norm = euclidean_norm(diff);
    for(int i = 0; i<start.size(); i++){
        diff[i] = start[i] + diff[i]*dist/norm;
    }
    return diff;
}