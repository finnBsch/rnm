//
// Created by finn on 27.05.21.
//

#include "rrt_node.h"
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
    if(this->parent_node->has_parent){
        auto temp_ = cost_two_joints(this->parent_node, this);
        this->cost_to_parent = std::get<0>(temp_) + std::get<1>(temp_)*params.steercost;
    }
    else{
        this->cost_to_parent = euclidean_dist_joint(this->parent_node->get_angles(), angles);
    }
    this->cost = cost_to_parent + this->parent_node->cost;
    angles_ = {angles[0],angles[1],angles[2],angles[3],angles[4],angles[5]};
}

tuple<float, float> rrt_node::cost_two_joints(rrt_node* init, rrt_node* stepped){
    float ang_cost = 0;
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
    ang_cost = abs(acos(ang_cost));
    return make_tuple(euclidean_dist_joint(init->get_angles(), stepped->get_angles()),ang_cost);
}