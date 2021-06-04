//
// Created by finn on 27.05.21.
//

#include "rrt.h"

/* Performance improvements:
 * euclidean dist: remove redudant array calls, implement euclidean dist sqrd (no need for sqrt)
 * generally decrease amount of array at(x) calls
 *
 * possible things to do:
 * do not insert near nodes to list but check in the loop for smalled node -> 10% of the whole time is used to insert
                                                                                                        nodes to vector*/

rrt::rrt(Point start_point, Point goal_point, rrt_params params) {
    int size = 3; // Dimensionality (rows)
    Eigen::MatrixXd covar(size,size);
    covar << 0.3, 0, 0,
            0, 0.3, 0,
            0, 0, 0.15;
    sampler = new normal_random_variable {covar};
    // save params
    this->goal_point = goal_point;
    this->start_point = start_point;
    this->params = params;
    // init start node
    start_node = new rrt_node(this->start_point);
    auto id = return_grid_id(start_point);
    // initialize grid
    // TODO Find faster way of saving stuff than nested vectors. Eigen?
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
        //sample_point = random_point(params.x_range, params.y_range, params.z_range);
        auto sampled = sampler->operator()();
        sampled[0] += goal_point[0];
        sampled[1] += goal_point[1];
        sampled[2] += goal_point[2];

        while(sampled[0] < params.x_range[0] || sampled[0] > params.x_range[1] || sampled[1] < params.y_range[0] || sampled[1] > params.y_range[1]
        || sampled[2] < params.z_range[0] || sampled[2] > params.z_range[1]){
            sampled = sampler->operator()();
            sampled[0] += goal_point[0];
            sampled[1] += goal_point[1];
            sampled[2] += goal_point[2];
        }
        sample_point = {(float)sampled[0], (float)sampled[1], (float)sampled[2]};
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
    if(euclidean_dist(new_node->get_pos(), goal_point)<0.05){
        goal_node = new_node;
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
    // precalculate loop limits to decrease array accesses
    int id_x = base_id[0];
    int id_y = base_id[1];
    int id_z = base_id[2];
    int num_x = num_cells[0];
    int num_y = num_cells[1];
    int num_z = num_cells[2];
    int radius = 0;
    float min_dist = -1;
    while(no_node_found){
        int cap_x = min(id_x+radius,num_x-1);
        int cap_y = min(id_y+radius,num_y-1);
        int cap_z = min(id_z+radius,num_z-1);
        for(int x = id_x-radius; x<=cap_x; x++){
            for(int y = id_y-radius; y<=cap_y; y++){
                for(int z = id_z-radius; z<=cap_z; z++){
                    if(abs(id_z-z)==radius || abs(id_y-y)==radius || abs(id_x-x)==radius){
                        if(x>= 0 && x<num_x && y>= 0 && y<num_y && z>= 0 && z<num_z) {
                            if (!grid.at(x).at(y).at(z).empty()) {
                                no_node_found = false;
                                for(rrt_node* c_node:grid.at(x).at(y).at(z)) {
                                    float current_dist = euclidean_dist_sqrd(c_node->get_pos(), relative_to);
                                    if (current_dist < min_dist || min_dist == -1) {
                                        min_dist = current_dist;
                                        nearest_node = c_node;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        radius++;
    }
    for(int x = id_x-radius; x<=id_x+radius; x++){
        for(int y = id_y-radius; y<=id_y+radius; y++){
            for(int z = id_z-radius; z<=id_z+radius; z++){
                if(abs(id_z-z)==radius || abs(id_y-y)==radius || abs(id_x-x)==radius){
                    if(x>= 0 && x<num_x && y>= 0 && y<num_y && z>= 0 && z<num_z) {
                        if (!grid.at(x).at(y).at(z).empty()) {
                            for(rrt_node* c_node:grid.at(x).at(y).at(z)) {
                                float current_dist = euclidean_dist_sqrd(c_node->get_pos(), relative_to);
                                if (current_dist < min_dist || min_dist == -1) {
                                    min_dist = current_dist;
                                    nearest_node = c_node;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return nearest_node;
}

vector<array<float, 3>> rrt::return_goal_path() {
    rrt_node* current_node = goal_node;
    vector<array<float, 3>> goal_path;
    array<float,3> point;
    do {
        point = current_node->get_pos();
        goal_path.push_back(point);
        current_node = current_node->get_parent();
    } while(current_node->get_parent() != nullptr);
    point = current_node->get_pos();
    goal_path.push_back(point);

    return goal_path;
}
