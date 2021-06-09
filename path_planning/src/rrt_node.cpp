//
// Created by finn on 27.05.21.
//

#include "rrt_node.h"
rrt_node::rrt_node(const joint_angles angles, const Point position):angles_(6) {
    this->position = position;
    this->angles = angles;
    this->cost = euclidean_dist_joint(this->angles, this->angles);
    this->cost_to_parent = cost;
    angles_ = {angles[0],angles[1],angles[2],angles[3],angles[4],angles[5]};
}
rrt_node::rrt_node(const joint_angles angles, const Point position, rrt_node* parent_node):angles_(6) {
    this->position = position;
    this->angles = angles;
    this->parent_node = parent_node;
    this->parent_node->add_child(this);
    this->cost_to_parent = euclidean_dist_joint(this->angles, this->parent_node->angles);
    this->cost = cost_to_parent + this->parent_node->cost;
    angles_ = {angles[0],angles[1],angles[2],angles[3],angles[4],angles[5]};
}
