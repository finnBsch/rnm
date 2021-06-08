//
// Created by finn on 27.05.21.
//

#ifndef SRC_RRT_NODE_H
#define SRC_RRT_NODE_H
#include "utility.h"

class rrt_node {
private:
    Point position;
    joint_angles angles;
    vector<float> angles_;
    rrt_node* parent_node = nullptr;
    list<rrt_node*> children;
public:
    rrt_node(const joint_angles angles, const Point position);
    rrt_node(const joint_angles angles, const Point position, rrt_node* parent_node);
    joint_angles & get_angles(){
        return angles;
    };
    Point& get_position(){
        return position;
    };
    vector<float>* get_angles_flann(){
        return &angles_;
    };
    rrt_node* get_parent(){
        return parent_node;
    };
};

#endif  // SRC_RRT_NODE_H
