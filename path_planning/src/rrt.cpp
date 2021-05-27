//
// Created by finn on 27.05.21.
//

#include "rrt.h"


rrt::rrt(Point start_point, Point goal_point,rrt_params params) {

    // save params
    this->goal_point = goal_point;
    this->start_point = start_point;
    this->params = params;
    // init start node
    start_node = new rrt_node(this->start_point);
    auto id = return_grid_id(start_point);
    // initialize grid
    vector<rrt_node*> dummy;
    for(int x = 0; x<ceil((params.x_range[1]-params.x_range[0])/params.grid_size)+1; x++){
        vector<vector<vector<rrt_node*>>> dummy_x;
        for(int y = 0; y<ceil((params.y_range[1]-params.y_range[0])/params.grid_size)+1; y++){
            vector<vector<rrt_node*>> dummy_y;
            for(int z = 0; z<ceil((params.z_range[1]-params.z_range[0])/params.grid_size)+1; z++){
                dummy_y.push_back(dummy);
            }
            dummy_x.push_back(dummy_y);
        }
        grid.push_back(dummy_x);
    }
    num_cells.at(0) = ceil((params.x_range[1]-params.x_range[0])/params.grid_size)+1;
    num_cells.at(1) = ceil((params.y_range[1]-params.y_range[0])/params.grid_size)+1;
    num_cells.at(2) = ceil((params.z_range[1]-params.z_range[0])/params.grid_size)+1;
    grid.at(id[0]).at(id[1]).at(id[2]).push_back(start_node);
}
tuple<bool, array<Point, 2>> rrt::expand() {
    rrt_node* nearest_node;
    rrt_node* new_node;
    Point sample_point;
    Point stepped_point;
    bool n_feasible = true;
    while(n_feasible) {
        sample_point = random_point(params.x_range, params.y_range, params.z_range);
        nearest_node = findNearestNode(sample_point);
        stepped_point = step_forward(nearest_node->get_pos(), sample_point, params.step_size);
        if(stepped_point[0]>params.x_range[0] && stepped_point[0]<params.x_range[1] &&
                stepped_point[1]>params.y_range[0] && stepped_point[1]<params.y_range[1] &&
                stepped_point[2]>params.z_range[0] && stepped_point[2]<params.z_range[1]){
            n_feasible = false;
        }
    }
    // TODO: Collision check, feasibility check
    new_node = new rrt_node(stepped_point, nearest_node);
    array<int, 3> id = return_grid_id(stepped_point);
    grid.at(id[0]).at(id[1]).at(id[2]).push_back(new_node);
    if(euclidean_dist(new_node->get_pos(), goal_point)<0.02){
        return make_tuple(true, (array<Point, 2>){new_node->get_parent()->get_pos(), new_node->get_pos()});
    }
    // TODO repeat until new node in case of collision
    return make_tuple(false, (array<Point, 2>){new_node->get_parent()->get_pos(), new_node->get_pos()});

}

array<int, 3> rrt::return_grid_id(Point point) {
    int id_x = floor((float)(point[0] - params.x_range[0])/params.grid_size);
    int id_y = floor((float)(point[1] - params.y_range[0])/params.grid_size);
    int id_z = floor((float)(point[2] - params.z_range[0])/params.grid_size);
    return (array<int, 3>){id_x, id_y, id_z};
}

rrt_node *rrt::findNearestNode(Point relative_to) {
    bool no_node_found = true;
    vector<rrt_node*> near_nodes;
    rrt_node* nearest_node;
    array<int, 3> base_id = return_grid_id(relative_to);
    int radius = 0;
    while(no_node_found){
        for(int x = base_id[0]-radius; x<=base_id[0]+radius; x++){
            for(int y = base_id[1]-radius; y<=base_id[1]+radius; y++){
                for(int z = base_id[2]-radius; z<=base_id[2]+radius; z++){
                    if(abs(base_id[2]-z)==radius || abs(base_id[1]-y)==radius || abs(base_id[0]-x)==radius){
                        if(x>= 0 && x<num_cells[0] && y>= 0 && y<num_cells[1] && z>= 0 && z<num_cells[2]) {
                            if (!grid.at(x).at(y).at(z).empty()) {
                                no_node_found = false;
                                near_nodes.insert(near_nodes.end(), grid.at(x).at(y).at(z).begin(),
                                                  grid.at(x).at(y).at(z).end());
                            }
                        }
                    }
                }
            }
        }
        radius++;
    }
    for(int x = base_id[0]-radius; x<=base_id[0]+radius; x++){
        for(int y = base_id[1]-radius; y<=base_id[1]+radius; y++){
            for(int z = base_id[2]-radius; z<=base_id[2]+radius; z++){
                if(abs(base_id[2]-z)==radius || abs(base_id[1]-y)==radius || abs(base_id[0]-x)==radius){
                    if(x>= 0 && x<num_cells[0] && y>= 0 && y<num_cells[1] && z>= 0 && z<num_cells[2]) {
                        if (!grid.at(x).at(y).at(z).empty()) {
                            near_nodes.insert(near_nodes.end(), grid.at(x).at(y).at(z).begin(),
                                              grid.at(x).at(y).at(z).end());
                        }
                    }
                }
            }
        }
    }
    float min_dist = -1;
    for(rrt_node* c_node:near_nodes){
        float current_dist = euclidean_dist(c_node->get_pos(), relative_to);
        if( current_dist < min_dist|| min_dist == -1){
            min_dist = current_dist;
            nearest_node = c_node;
        }
    }
    return nearest_node;
}
