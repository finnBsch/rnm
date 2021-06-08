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



Matrix<double, 4, 4> get_transformationmatrix(const double theta, const double a, const double d, const double alpha){
    Matrix<double, 4, 4> ret_mat;
    double st = sin(theta);
    double ca = cos(alpha);
    double sa = sin(alpha);
    double ct = cos(theta);
    ret_mat << ct, -st, 0, a,
            st * ca, ct*ca, -sa, -d * sa,
            st * sa, ct*sa, ca, d * ca,
            0, 0, 0, 1;
    return ret_mat;
}
void get_transformationmatrix2(const double theta, const double a, const double d, const double alpha, Matrix<double, 4, 4>& in){
    double st = sin(theta);
    double ca = cos(alpha);
    double sa = sin(alpha);
    double ct = cos(theta);
    in(0,0) = ct;
    in(0,1) = -st;
    in(0,2) = 0;
    in(0,3) = a;
    in(1, 0) = st*ca;
    in(1, 1) = ct*ca;
    in(1,2 ) = -sa;
    in(1,3) = -d*sa;
    in(2,0) = st*sa;
    in(2,1) = ct*sa;
    in(2, 2) = ca;
    in(2,3) = d*ca;
    in(3,0) = 0;
    in(3,1) = 0;
    in(3,2) = 0;
    in(3,3) = 1;
}

Point rrt::get_end_effector(joint_angles angles){
    boost::array<MatrixXd, 8> a_;
    for(int i  = 0; i<7; i++){
        a_.at(i) = get_transformationmatrix(angles[i], a(i), d(i), alpha(i));
    }
    a_.at(7) = get_transformationmatrix(0, a(7), d(7), alpha(7));
    VectorXd in(4);
    in << 0, 0, 0, 1;
    VectorXd out = a_.at(0)*a_.at(1)*a_.at(2)*a_.at(3)*a_.at(4)*a_.at(5)*a_.at(6)*a_.at(7)*in;
    //std::cout << "End_pos: " << "\n" << "x: " << out(0) << "\n" << "y: " << out(1) << "\n" << "z: " << out(2) << "\n";
    return (Point){out[0], out[1], out[2]};
}

rrt::rrt(Point start_point, Point goal_point, rrt_params params, joint_angles init):kdtree(flann::KDTreeIndexParams()) {
    a << 0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0;
    d << 0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107;
    alpha << 0, -M_PI/2, M_PI/2, M_PI/2, -M_PI/2, M_PI/2, M_PI/2, 0;
    int size = 3; // Dimensionality (rows)
    kdtree = flann::Index<flann::L2_Simple<double>>(
            flann::KDTreeIndexParams());
    Eigen::MatrixXd covar(size,size);
    float cov = 5;
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
    std::vector<double> data(3);
    point_to_flann(start_node->get_pos(), data.data());
    kdtree.buildIndex(
            flann::Matrix<double>(data.data(), 1, 3));
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
            sample_point[i] =  sampled[i];
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
    new_node = new rrt_node(get_end_effector(new_angles), nearest_node, new_angles);
    kdtree.addPoints(flann::Matrix<double>(new_node->get_pos_flann()->data(), 1, 3));
    //nodemap.insert(std::pair<Point, rrt_node*>(new_node->get_pos(), new_node));
    all_nodes.push_back(new_node);
    if(euclidean_dist(new_node->get_pos(), goal_point)<0.05){
        goal_node = new_node;
        return make_tuple(true, (array<Point, 2>){new_node->get_parent()->get_pos(), new_node->get_pos()});
    }
    // TODO repeat until new node in case of collision
    return make_tuple(false, (array<Point, 2>){new_node->get_parent()->get_pos(), new_node->get_pos()});

}


rrt_node *rrt::findNearestNode(Point& relative_to) {
    flann::Matrix<double> query;
    std::vector<double> data(3);
    point_to_flann(relative_to, data.data());
    query = flann::Matrix<double>(data.data(), 1,
                                 relative_to.size());
    std::vector<int> i(query.rows);
    flann::Matrix<int> indices(new int[query.rows], query.rows, 1);
    std::vector<double> d(query.rows);
    flann::Matrix<double> dists(new double[query.rows], query.rows, 1);

    int n = kdtree.knnSearch(query, indices, dists, 1, flann::SearchParams());

    //Point point;
    //point = flann_to_point(kdtree.getPoint(indices[0][0]));
    return all_nodes[indices[0][0]];
}

vector<tuple<Point, joint_angles>> rrt::return_goal_path() {
    rrt_node* current_node = goal_node;
    vector<tuple<Point, joint_angles>> goal_path;
    do {
        goal_path.push_back(make_tuple(current_node->get_pos(), current_node->get_angle()));
        current_node = current_node->get_parent();
    } while(current_node->get_parent() != nullptr);
    goal_path.push_back(make_tuple(current_node->get_pos(), current_node->get_angle()));
    return goal_path;
}

boost::array<double, 7> rrt::generateSuccessor(rrt_node *root, Point goal_point) {
    //TODO  precalculate matrices for forward kinematics
    boost::array<double, 7> best_set;
    auto arr = root->get_angle();
    joint_angles arr2;
    if(root->get_parent()) {
        arr2 = root->get_parent()->get_angle();
    }
    else{
        arr2 = arr;
    }
    float min = -1;
    boost::array<double, 7> temp;
    temp[6] = arr[6];

    Matrix<double, 4, 4> joint0;
    Matrix<double, 4, 4> joint1;
    Matrix<double, 4, 4> joint2;
    Matrix<double, 4, 4> joint3;
    Matrix<double, 4, 4> joint4;
    Matrix<double, 4, 4> joint5;
    Matrix<double, 4, 4> joint6;
    Matrix<double, 4, 4> joint7;
    get_transformationmatrix2(0, a(6), d(6), alpha(6), joint6);
    get_transformationmatrix2(0, a(7), d(7), alpha(7), joint7);
    Matrix<double, 4, 4> last_product = joint6*joint7;
    Matrix<double, 4, 1> in;
    Matrix<double, 4, 1> out;
    int steps = 1;
    float fac = 1.3;
    in << 0, 0, 0, 1;
    for(int i = -steps; i<=steps; i++){
        temp[0] = arr[0]+ (arr[0]-arr2[0] + i*fac*M_PI/180)/2;
        get_transformationmatrix2(temp[0], a(0), d(0), alpha(0), joint0);
        for(int j = -steps; j<=steps; j++){
            temp[1] = arr[1]+ (arr[1] - arr2[1] + j*fac*1.2*M_PI/180)/2;
            get_transformationmatrix2(temp[1], a(1), d(1), alpha(1), joint1);
            joint1 = joint0*joint1;
            for(int k = -steps; k<=steps; k++){
                temp[2] =arr[2]+ (arr[2] - arr2[2] + k*fac*1.4*M_PI/180)/2;
                get_transformationmatrix2(temp[2], a(2), d(2), alpha(2), joint2);
                joint2 = joint1*joint2;
                for(int  l= -steps; l<=steps; l++){
                    temp[3] = arr[3]+ (arr[3]-arr2[3] + l*fac*1.6*M_PI/180)/2;
                    get_transformationmatrix2(temp[3], a(3), d(3), alpha(3), joint3);
                    joint3= joint2*joint3;
                    for(int m = -steps; m<=steps; m++){
                        temp[4] = arr[4]+  (arr[4] - arr2[4] + m*fac*1.8*M_PI/180)/2;
                        get_transformationmatrix2(temp[4], a(4), d(4), alpha(4), joint4);
                        joint4=joint3*joint4;
                        for(int n = -steps; n<=steps; n++){
                            temp[5] =arr[5]+ (arr[5] - arr2[5] + n*2*M_PI/180)/2;
                            get_transformationmatrix2(temp[5], a(5), d(5), alpha(5), joint5);
                            joint5=joint4*joint5;
                            out = joint5*last_product*in;
                            float temp_ = euclidean_dist_sqrd_boost((boost::array<double, 3>){out[0], out[1], out[2]}, goal_point);
                            if(min ==-1 || temp_ < min){
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
