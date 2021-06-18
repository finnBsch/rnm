//
// Created by finn on 27.05.21.
//

#include "rrt_node.h"
#include "ros/ros.h"

rrt_node::rrt_node(const joint_angles angles, const Point position,rrt_params params):angles_(6) {
    this->params = params;
    this->position = position;
    this->angles = angles;
    this->cost = euclidean_dist_joint(this->angles, this->angles);
    this->cost_to_parent = cost;
    angles_ = {angles[0],angles[1],angles[2],angles[3],angles[4],angles[5]};
    has_parent = false;
}

rrt_node::rrt_node(const joint_angles angles, const Point position,rrt_params params, rrt_node* parent_node):angles_(6) {
    this->params = params;
    this->position = position;
    this->angles = angles;
    this->parent_node = parent_node;
    this->parent_node->add_child(this);
    this->cost_to_parent = euclidean_dist_joint(this->parent_node->get_angles(), angles);
    this->cost = cost_to_parent + this->parent_node->cost;
    angles_ = {angles[0],angles[1],angles[2],angles[3],angles[4],angles[5]};
}

tuple<double, double> rrt_node::cost_two_joints(rrt_node* init, rrt_node* stepped){
    double ang_cost = 0;
    joint_angles A;
    joint_angles B;
    joint_angles pre = init->get_parent()->get_angles();
    joint_angles init_ang = init->get_angles();
    joint_angles stepped_ang = stepped->get_angles();
    for(int i = 0; i<A.size(); i++){
        A[i] = init_ang[i]-pre[i];
        B[i] = stepped_ang[i] -init_ang[i];
        ang_cost+=A[i]*B[i];
    }
    ang_cost=ang_cost/(euclidean_norm(A)*euclidean_norm(B));
    if(ang_cost<=-1.0){
        ang_cost = M_PI*M_PI;
    }
    else if(ang_cost>=1.0){
        ang_cost = 0;
    }
    else {
        ang_cost = pow(acos(ang_cost), 2);
    }
    // todo: check if +- 180°
    return make_tuple(euclidean_dist_joint(init->get_angles(), stepped->get_angles()),ang_cost);
}