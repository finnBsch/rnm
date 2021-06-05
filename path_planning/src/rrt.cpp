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
    int size = 6;
    Eigen::MatrixXd covar(size,size);
    covar << 0.3, 0, 0, 0, 0, 0,
            0, 0.3, 0, 0, 0, 0,
            0, 0, 0.3, 0, 0, 0,
            0, 0, 0, 0.3, 0, 0,
            0, 0, 0, 0, 0.3, 0;
    // save params
    this->goal_point = goal_point;
    this->start_point = start_point;
    this->params = params;
    // init start node
    start_node = new rrt_node(this->start_point);
    auto id = return_grid_id(start_point);
    // initialize grid
    vector<rrt_node*> dummy;
    for(int a = 0; a<ceil((params.joint_ranges[0][1]-params.joint_ranges[0][0])/params.grid_size)+1; a++){
        vector<vector<vector<vector<vector<vector<rrt_node*>>>>>> dummy_a;
        dummy_a.reserve(ceil((params.joint_ranges[1][1]-params.joint_ranges[1][0])/params.grid_size)+1);
        for(int b = 0; b<ceil((params.joint_ranges[1][1]-params.joint_ranges[1][0])/params.grid_size)+1; b++){
            vector<vector<vector<vector<vector<rrt_node*>>>>> dummy_b;
            dummy_b.reserve(ceil((params.joint_ranges[2][1]-params.joint_ranges[2][0])/params.grid_size)+1);
            for(int c = 0; c<ceil((params.joint_ranges[2][1]-params.joint_ranges[2][0])/params.grid_size)+1; c++){
                vector<vector<vector<vector<rrt_node*>>>> dummy_c;
                dummy_c.reserve(ceil((params.joint_ranges[3][1]-params.joint_ranges[3][0])/params.grid_size)+1);
                for(int d = 0; d<ceil((params.joint_ranges[3][1]-params.joint_ranges[3][0])/params.grid_size)+1; d++){
                    vector<vector<vector<rrt_node*>>> dummy_d;
                    dummy_d.reserve(ceil((params.joint_ranges[4][1]-params.joint_ranges[4][0])/params.grid_size)+1);
                    for(int e = 0; e<ceil((params.joint_ranges[4][1]-params.joint_ranges[4][0])/params.grid_size)+1; e++){
                        vector<vector<rrt_node*>> dummy_e;
                        dummy_e.reserve(ceil((params.joint_ranges[5][1]-params.joint_ranges[5][0])/params.grid_size)+1);
                        for(int f = 0; f<ceil((params.joint_ranges[5][1]-params.joint_ranges[5][0])/params.grid_size)+1; f++){
                            dummy_e.push_back(dummy);
                        }
                        dummy_d.push_back(dummy_e);
                    }
                    dummy_c.push_back(dummy_d);
                }
                dummy_b.push_back(dummy_c);
            }
            dummy_a.push_back(dummy_b);
        }
        grid.push_back(dummy_a);
    }
    for(int i = 0; i<num_cells.size(); i++){
        num_cells.at(i) = ceil((params.joint_ranges[i][1]-params.joint_ranges[i][0])/params.grid_size)+1;
    }
    grid.at(id[0]).at(id[1]).at(id[2]).at(id[3]).at(id[4]).at(id[5]).push_back(start_node);
}
tuple<bool, array<Point, 2>> rrt::expand() {
    rrt_node* nearest_node;
    rrt_node* new_node;
    Point sample_point;
    Point stepped_point;
    bool n_feasible = true;
    while(n_feasible) {
        //sample_point = random_point(params.x_range, params.y_range, params.z_range);
        sample_point = random_point(params.joint_ranges);
        nearest_node = findNearestNode(sample_point);
        stepped_point = step_forward(nearest_node->get_pos(), sample_point, params.step_size);
        for(int i = 0; i<stepped_point.size();i++){
            if(stepped_point[0]>params.joint_ranges[i][0] && stepped_point[0]<params.joint_ranges[i][1])
            {
                n_feasible = false;
            }
            else{
                n_feasible = true;
                break;
            }
        }
    }
    // TODO: Collision check, feasibility check
    new_node = new rrt_node(stepped_point, nearest_node);
    array<int, 6> id = return_grid_id(stepped_point);
    grid.at(id[0]).at(id[1]).at(id[2]).at(id[3]).at(id[4]).at(id[5]).push_back(new_node);
    num_nodes++;
    if(euclidean_dist(new_node->get_pos(), goal_point)<0.05){
        goal_node = new_node;
        return make_tuple(true, (array<Point, 2>){new_node->get_parent()->get_pos(), new_node->get_pos()});
    }
    // TODO repeat until new node in case of collision
    return make_tuple(false, (array<Point, 2>){new_node->get_parent()->get_pos(), new_node->get_pos()});

}

array<int, 6> rrt::return_grid_id(Point point) {
    array<int, 6> id;
    for(int i = 0; i < point.size(); i++){
        id[i] = floor((float)(point[0] - params.joint_ranges[i][0])/params.grid_size);
    }
    return id;
}

rrt_node *rrt::findNearestNode(Point relative_to) {
    bool no_node_found = true;
    vector<rrt_node*> near_nodes;
    rrt_node* nearest_node;
    array<int, 6> base_id = return_grid_id(relative_to);
    // precalculate loop limits to decrease array accesses
    int id_a = base_id[0];
    int id_b = base_id[1];
    int id_c = base_id[2];
    int id_d = base_id[3];
    int id_e = base_id[4];
    int id_f = base_id[5];
    int num_a = num_cells[0];
    int num_b = num_cells[1];
    int num_c = num_cells[2];
    int num_d = num_cells[3];
    int num_e = num_cells[4];
    int num_f = num_cells[5];
    int radius = 0;
    float min_dist = -1;
    while(no_node_found){
        int cap_a = min(id_a+radius,num_a-1);
        int cap_b = min(id_b+radius,num_b-1);
        int cap_c = min(id_c+radius,num_c-1);
        int cap_d = min(id_d+radius,num_d-1);
        int cap_e = min(id_e+radius,num_e-1);
        int cap_f = min(id_f+radius,num_f-1);
        for(int a = id_a-radius; a<=cap_a; a++){
            for(int b = id_b-radius; b<=cap_b; b++){
                for(int c = id_c-radius; c<=cap_c; c++){
                    for(int d = id_d-radius; d<=cap_d; d++){
                        for(int e = id_e-radius; e<=cap_e; e++){
                            for(int f = id_f-radius; f<=cap_f; f++){
                                if(abs(id_f-f)==radius || abs(id_e-e)==radius || abs(id_d-d)==radius || abs(id_c-c)==radius || abs(id_b-b)==radius || abs(id_a-a)==radius){
                                    if(a>= 0 && a<num_a && b>= 0 && b<num_b && c>= 0 && c<num_c &&
                                       d>= 0 && d<num_d && e>= 0 && e<num_e && f>= 0 && f<num_f) {
                                        if (!grid.at(a).at(b).at(c).at(d).at(e).at(f).empty()) {
                                            no_node_found = false;
                                            for (rrt_node *c_node:grid.at(a).at(b).at(c).at(d).at(e).at(f)) {
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
                }
            }
        }
        radius++;
    }
    int cap_a = min(id_a+radius,num_a-1);
    int cap_b = min(id_b+radius,num_b-1);
    int cap_c = min(id_c+radius,num_c-1);
    int cap_d = min(id_d+radius,num_d-1);
    int cap_e = min(id_e+radius,num_e-1);
    int cap_f = min(id_f+radius,num_f-1);
    for(int a = id_a-radius; a<=cap_a; a++){
        for(int b = id_b-radius; b<=cap_b; b++){
            for(int c = id_c-radius; c<=cap_c; c++) {
                for (int d = id_d - radius; d <= cap_d; d++) {
                    for (int e = id_e - radius; e <= cap_e; e++) {
                        for (int f = id_f - radius; f <= cap_f; f++) {
                            if (abs(id_f - f) == radius || abs(id_e - e) == radius || abs(id_d - d) == radius ||
                                abs(id_c - c) == radius || abs(id_b - b) == radius || abs(id_a - a) == radius) {
                                if (a >= 0 && a < num_a && b >= 0 && b < num_b && c >= 0 && c < num_c &&
                                    d >= 0 && d < num_d && e >= 0 && e < num_e && f >= 0 && f < num_f) {
                                    if (!grid.at(a).at(b).at(c).at(d).at(e).at(f).empty()) {
                                        for (rrt_node *c_node:grid.at(a).at(b).at(c).at(d).at(e).at(f)) {
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
            }
        }
    }
    return nearest_node;
}

vector<Point> rrt::return_goal_path() {
    rrt_node* current_node = goal_node;
    vector<Point> goal_path;
    Point point;
    do {
        point = current_node->get_pos();
        goal_path.push_back(point);
        current_node = current_node->get_parent();
    } while(current_node->get_parent() != nullptr);
    point = current_node->get_pos();
    goal_path.push_back(point);
    return goal_path;
}
