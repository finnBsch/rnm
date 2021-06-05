//
// Created by finn on 27.05.21.
//

#include "rrt_node.h"
rrt_node::rrt_node(Point position) {
    this->position = position;
    pos = {position[0],position[1],position[2],position[3],position[4],position[5]};
}
rrt_node::rrt_node(Point position, rrt_node* parent_node) {
    this->position = position;
    this->parent_node = parent_node;
}
