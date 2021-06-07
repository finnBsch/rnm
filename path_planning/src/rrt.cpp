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
using namespace Eigen;
static VectorXd a(8);
static VectorXd d(8);
static VectorXd alpha(8);

MatrixXd get_transformationmatrix(const float theta, const float a, const float d, const float alpha){
    MatrixXd ret_mat(4,4);
    ret_mat << cos(theta), -sin(theta), 0, a,
            sin(theta) * cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d * sin(alpha),
            sin(theta) * sin(alpha), cos(theta)*sin(alpha), cos(alpha), d * cos(alpha),
            0, 0, 0, 1;
    return ret_mat;
}

bool get_end_effector(forward_kin::get_endeffector::Request  &req,
                      forward_kin::get_endeffector::Response &res) {
    std::array<MatrixXd, 8> a_;
    for(int i  = 0; i<7; i++){
        a_.at(i) = get_transformationmatrix(req.joint_angles[i], a(i), d(i), alpha(i));
    }
    a_.at(7) = get_transformationmatrix(0, a(7), d(7), alpha(7));
    VectorXd in(4);
    in << 0, 0, 0, 1;
    VectorXd out = a_.at(0)*a_.at(1)*a_.at(2)*a_.at(3)*a_.at(4)*a_.at(5)*a_.at(6)*a_.at(7)*in;
    res.end_effector_pos =  {out[0], out[1], out[2]};
    //std::cout << "End_pos: " << "\n" << "x: " << out(0) << "\n" << "y: " << out(1) << "\n" << "z: " << out(2) << "\n";
    return true;
}

rrt::rrt(Point start_point, Point goal_point, rrt_params params, joint_angles init):kdtree(flann::KDTreeIndexParams()) {
    int size = 3; // Dimensionality (rows)
    kdtree = flann::Index<flann::L2_Simple<float>>(
            flann::KDTreeIndexParams());
    Eigen::MatrixXd covar(size,size);
    float cov = 0.18;
    covar << cov, 0, 0,
            0, cov, 0,
            0, 0, cov/2;
    sampler = new normal_random_variable {covar};
    // save params
    this->goal_point = goal_point;
    this->start_point = start_point;
    this->params = params;
    // init start node
    start_node = new rrt_node(this->start_point, init);
    std::vector<float> data(3);
    point_to_flann(start_node->get_pos(), data.data());
    kdtree.buildIndex(
            flann::Matrix<float>(data.data(), 1, 3));
    all_nodes.push_back(start_node);
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
            for(int i = 0; i<sampled.size(); i++){
                if(sampled[i] < params.ranges[i][0] || sampled[i] > params.ranges[i][1] ){
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
        //sample_point = random_point(params.ranges);
        nearest_node = findNearestNode(sample_point);
        stepped_point = step_forward(nearest_node->get_pos(), sample_point, params.step_size);
        for(int i = 0; i<stepped_point.size();i++){
            if(stepped_point[i]>params.ranges[i][0] && stepped_point[i] < params.ranges[i][1])
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

    //srv.request.joint_angles =
    auto new_angles = generateSuccessor(nearest_node, stepped_point);
    new_node = new rrt_node(srv.response.end_effector_pos, nearest_node, new_angles);
    kdtree.addPoints(flann::Matrix<float>(new_node->get_pos_flann()->data(), 1, 3));
    //nodemap.insert(std::pair<Point, rrt_node*>(new_node->get_pos(), new_node));
    all_nodes.push_back(new_node);
    num_nodes++;
    if(euclidean_dist(new_node->get_pos(), goal_point)<0.01){
        goal_node = new_node;
        return make_tuple(true, (array<Point, 2>){new_node->get_parent()->get_pos(), new_node->get_pos()});
    }
    // TODO repeat until new node in case of collision
    return make_tuple(false, (array<Point, 2>){new_node->get_parent()->get_pos(), new_node->get_pos()});

}


rrt_node *rrt::findNearestNode(Point& relative_to) {
    flann::Matrix<float> query;
    std::vector<float> data(3);
    point_to_flann(relative_to, data.data());
    query = flann::Matrix<float>(data.data(), 1,
                                 relative_to.size());
    std::vector<int> i(query.rows);
    flann::Matrix<int> indices(new int[query.rows], query.rows, 1);
    std::vector<float> d(query.rows);
    flann::Matrix<float> dists(new float[query.rows], query.rows, 1);

    int n = kdtree.knnSearch(query, indices, dists, 1, flann::SearchParams());

    //Point point;
    //point = flann_to_point(kdtree.getPoint(indices[0][0]));
    return all_nodes[indices[0][0]];
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

boost::array<double, 7> rrt::generateSuccessor(rrt_node *root, Point goal_point) {
    boost::array<double, 7> best_set;
    auto arr = root->get_angle();
    float min = -1;
    boost::array<double, 7> temp;
    temp[6] = 0;
    for(int i = -2; i<=2; i++){
        temp[0] = arr[0]+ i*0.5*M_PI/180;
        for(int j = -2; j<=2; j++){
            temp[1] = arr[1]+ j*0.5*M_PI/180;
            for(int k = -2; k<=2; k++){
                temp[2] =arr[2]+  k*0.5*M_PI/180;
                for(int  l= -2; l<=2; l++){
                    temp[3] = arr[3]+ l*0.5*M_PI/180;
                    for(int m = -2; m<=2; m++){
                        temp[4] = arr[4]+  m*0.5*M_PI/180;
                        for(int n = -2; n<=2; n++){
                            temp[5] =arr[5]+   n*0.5*M_PI/180;
                            float temp_ = euclidean_dist_sqrd_boost(srv.response.end_effector_pos, goal_point);
                            if(min ==-1 || euclidean_dist_sqrd_boost(srv.response.end_effector_pos, goal_point) < min){
                                min = temp_;
                                best_set = temp;
                            }
                        }
                    }
                }
            }
        }
    }
    return best_set;
}
