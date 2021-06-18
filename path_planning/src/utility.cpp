//
// Created by finn on 27.05.21.
//

#include "utility.h"
void point_to_flann(joint_angles p, double* data){
data[0] = p[0];
data[1] = p[1];
data[2] = p[2];
data[3] = p[3];
data[4] = p[4];
data[5] = p[5];
}
joint_angles flann_to_point(double* data){
return (joint_angles){data[0],data[1],data[2],data[3],data[4],data[5]};
}
double euclidean_dist(Point A, Point B){
    double distances;
    double total_distance = 0;
    for(int i = 0; i < A.size(); i++){
        distances = A[i] - B[i];
        total_distance += distances*distances;
    }
    return sqrt(total_distance);
}
Matrix<double, 6, 1> sample_unit_ball(){
    Matrix<double, 6, 1> r;
    for(int i = 0; i < r.size(); i++){
        r[i] = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
    }
    double norm = r.norm();
    double r2 = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
    for(int i = 0; i < r.size(); i++){
        r[i] = r[i]*r2/norm;
    }
    return r;
}
double euclidean_dist_sqrd_joint(joint_angles A, joint_angles B){
    double distances;
    double total_distance = 0;
    for(int i = 0; i < A.size(); i++){
        distances = abs(A[i] - B[i]);
        while(abs(distances)>M_PI){
            distances-=2*M_PI;
        }
        total_distance += distances*distances;
    }
    return total_distance;
}
double euclidean_dist_joint(joint_angles A, joint_angles B){
    double distances;
    double total_distance = 0;
    for(int i = 0; i < A.size(); i++){
        distances = abs(A[i] - B[i]);
        while(abs(distances)>M_PI){
            distances-=2*M_PI;
        }
        total_distance += distances*distances;
    }
    return sqrt(total_distance);
}
double euclidean_dist_sqrd(Point A, Point B){
    double distances;
    double total_distance = 0;
    for(int i = 0; i < A.size(); i++){
        distances = A[i] - B[i];
        total_distance += distances*distances;
    }
    return total_distance;
}
double euclidean_norm(joint_angles A){
    double distances;
    double total_distance = 0;
    for(int i = 0; i < A.size(); i++){
        distances = A[i];
        total_distance += distances*distances;
    }
    return sqrt(total_distance);
}

joint_angles random_point(array<array<double, 2>, 6> joint_ranges){
    joint_angles sampled;
    double LO;
    double HI;
    double sampled_angle;
    int i = 0;
    for(array<double, 2> range:joint_ranges){
        LO = range[0];
        HI = range[1];
        sampled_angle = LO + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(HI-LO)));
        sampled[i] = sampled_angle;
        i++;
    }
    return sampled;
}

joint_angles step_forward(joint_angles start, joint_angles goal, double dist){
    joint_angles diff;
    for(int i = 0; i<start.size(); i++){
        diff[i] = goal[i]-start[i];
    }
    double norm = euclidean_norm(diff);
    for(int i = 0; i<start.size(); i++){
        diff[i] = start[i] + diff[i]*dist/norm;
    }
    return diff;
}