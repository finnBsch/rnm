//
// Created by finn on 27.05.21.
//

#ifndef SRC_RRT_NODE_H
#define SRC_RRT_NODE_H
#include "utility.h"

class rrt_node {
private:
    Point position;
    vector<float> pos;
    rrt_node* parent_node = nullptr;
    list<rrt_node*> children;
public:
    rrt_node(Point position);
    rrt_node(Point position, rrt_node* parent_node);
    Point& get_pos(){
        return position;
    };
    vector<float>* get_pos_flann(){
        return &pos;
    };
    rrt_node* get_parent(){
        return parent_node;
    };
};

#endif  // SRC_RRT_NODE_H
