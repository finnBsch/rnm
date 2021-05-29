//
// Created by finn on 27.05.21.
//

#include "rrt_node.h"
rrt_node::rrt_node(Point position) {
    this->position = position;
}
rrt_node::rrt_node(Point position, rrt_node* parent_node) {
    this->position = position;
    this->parent_node = parent_node;
}
