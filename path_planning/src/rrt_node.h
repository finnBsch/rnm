//
// Created by finn on 27.05.21.
//

#ifndef SRC_RRT_NODE_H
#define SRC_RRT_NODE_H
#include "utility.h"
#include "ros/ros.h"
#include "bullet/btBulletCollisionCommon.h"

class rrt_node {
private:
    Point position;
    joint_angles angles;
    vector<float> angles_;
    rrt_node* parent_node = nullptr;
    list<rrt_node*> children;
    rrt_params params;
public:
    bool has_child = false;
    bool has_parent = true;
    tuple<float, float> cost_two_joints(rrt_node* init, rrt_node* stepped);
    rrt_node(const joint_angles angles, const Point position,rrt_params params);
    rrt_node(const joint_angles angles, const Point position,rrt_params params, rrt_node* parent_node);
    float cost = 0;
    float cost_to_parent = 0;
    void set_parent(rrt_node* parent, float cost, float cost_to_parent){
        this->parent_node->remove_child(this);
        this->parent_node = parent;
        this->parent_node->add_child(this);
        this->cost = cost;
        this->cost_to_parent = cost_to_parent;
        propagate_costs();
    }
    void propagate_costs(){
        for(auto child:children){
            child->cost = child->cost_to_parent + cost;
            if(child->cost < 0 || child->cost > 1000){
                ROS_INFO("WTF");
            }
            child->propagate_costs();
        }
    }
    void add_child(rrt_node* child){
        this->children.push_back(child);
        has_child = true;
    }
    void remove_child(rrt_node* child){
        this->children.remove(child);
    }
    joint_angles get_angles(){
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
