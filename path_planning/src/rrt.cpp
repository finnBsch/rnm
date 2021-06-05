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
    float cov = 0.1;
    covar << cov, 0, 0, 0, 0, 0,
            0, cov, 0, 0, 0, 0,
            0, 0, cov, 0, 0, 0,
            0, 0, 0, cov, 0, 0,
            0, 0, 0, 0, cov, 0,
            0, 0, 0, 0, 0, cov;
    sampler = new normal_random_variable {covar};
    // save params
    this->goal_point = goal_point;
    this->start_point = start_point;
    this->params = params;
    // init start node
    start_node = new rrt_node(this->start_point);
    kdtree.addPoints(flann::Matrix<float>(start_node->get_pos_flann()->data(), 1, 6));
    all_nodes.push_back(start_node);
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
        auto sampled = sampler->operator()();
        bool not_feasible = true;
        while(not_feasible){
            sampled = sampler->operator()();
            sampled[0] += goal_point[0];
            sampled[1] += goal_point[1];
            sampled[2] += goal_point[2];
            sampled[3] += goal_point[3];
            sampled[4] += goal_point[4];
            sampled[5] += goal_point[5];
            for(int i = 0; i<sampled.size(); i++){
                if(sampled[0] < params.joint_ranges[i][0] || sampled[0] > params.joint_ranges[i][1] ){
                    not_feasible = true;
                    break;
                }
                else{
                    not_feasible = false;
                }
            }
        }
        for(int i = 0; i<sampled.size(); i++){
            sample_point[i] =  (float)sampled[i];
        }
        //sample_point = random_point(params.joint_ranges);
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
    kdtree.addPoints(flann::Matrix<float>(new_node->get_pos_flann()->data(), 1, 6));
    nodemap.insert(std::pair<Point, rrt_node*>(new_node->get_pos(), &new_node));
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
    rrt_node* nearest_node = nullptr;
    flann::Matrix<float> query;
    std::vector<float> data(6);
    point_to_flann(relative_to, data.data());
    query = flann::Matrix<float>(data.data(), 1,
                                      sizeof(relative_to) / sizeof(0.0));
    std::vector<int> i(query.rows);
    flann::Matrix<int> indices(i.data(), query.rows, 1);
    std::vector<double> d(query.rows);
    flann::Matrix<double> dists(d.data(), query.rows, 1);

    int n = kdtree.knnSearch(query, indices, dists, 1, flann::SearchParams());

    Point point;
    point = flann_to_point(kdtree.getPoint(indices[0][0]));
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
