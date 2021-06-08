//
// Created by finn on 27.05.21.
//

#include "rrt_node.h"
rrt_node::rrt_node(const joint_angles angles, const Point position):angles_(6) {
    this->position = position;
    this->angles = angles;
    angles_ = {angles[0],angles[1],angles[2],angles[3],angles[4],angles[5]};
}
rrt_node::rrt_node(const joint_angles angles, const Point position, rrt_node* parent_node):angles_(6) {
    this->position = position;
    this->angles = angles;
    this->parent_node = parent_node;
    angles_ = {angles[0],angles[1],angles[2],angles[3],angles[4],angles[5]};
}
