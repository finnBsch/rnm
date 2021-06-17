//
// Created by finn on 27.05.21.
//

#include "rrt.h"


#define T 6
/* Performance improvements:
 * euclidean dist: remove redudant array calls, implement euclidean dist sqrd (no need for sqrt)
 * generally decrease amount of array at(x) calls
 *
 * possible things to do:
 * do not insert near nodes to list but check in the loop for smalled node -> 10% of the whole time is used to insert
                                                                                                        nodes to vector*/


void eig_to_bt(Matrix<float, 4, 4> eig, btMatrix3x3& bt){
    for(int i = 0; i<3; i++){
        for(int j = 0; j<3; j++){
            bt[i][j] = eig(i, j);
        }
    }
}
// return Transformation matrices from dvh
Matrix<float, 4, 4> get_transformationmatrix(const float theta, const float a, const float d, const float alpha){

    Matrix<float, 4, 4> ret_mat;
    float st = sin(theta);
    float ca = cos(alpha);
    float sa = sin(alpha);
    float ct = cos(theta);
    ret_mat << ct, -st, 0, a,
            st * ca, ct*ca, -sa, -d * sa,
            st * sa, ct*sa, ca, d * ca,
            0, 0, 0, 1;
    return ret_mat;
}
void get_transformationmatrix2(const float theta, const float a, const float d, const float alpha, Matrix<float, 4, 4>& in){
    float st = sin(theta);
    float ca = cos(alpha);
    float sa = sin(alpha);
    float ct = cos(theta);
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

// end effector pos from joint angles
Point rrt::get_end_effector(joint_angles angles){
    array<Matrix<float, 4, 4>, 8> a_;
    for(int i  = 0; i<T; i++){
        a_.at(i) = get_transformationmatrix(angles[i], a(i), d(i), alpha(i));
    }
    a_.at(6) = get_transformationmatrix(0, a(6), d(6), alpha(6));
    a_.at(7) = get_transformationmatrix(0, a(7), d(7), alpha(7));
    Matrix<float, 4, 1> in;
    in << 0, 0, 0, 1;
    Matrix<float, 4, 1> out = a_.at(0)*a_.at(1)*a_.at(2)*a_.at(3)*a_.at(4)*a_.at(5)*a_.at(6)*a_.at(7)*in;
    //std::cout << "End_pos: " << "\n" << "x: " << out(0) << "\n" << "y: " << out(1) << "\n" << "z: " << out(2) << "\n";
    return (Point){out[0], out[1], out[2]};
}

// end-effector normal from joint angles
Point rrt::get_end_effector_normal(joint_angles angles){
    array<Matrix<float, 4, 4>, 8> a_;
    for(int i  = 0; i<T; i++){
        a_.at(i) = get_transformationmatrix(angles[i], a(i), d(i), alpha(i));
    }
    a_.at(6) = get_transformationmatrix(0, a(6), d(6), alpha(6));
    a_.at(7) = get_transformationmatrix(0, a(7), d(7), alpha(7));
    Matrix<float, 4, 1> in;
    in << 0, 0, 1, 0;
    Matrix<float, 4, 1> out = a_.at(0)*a_.at(1)*a_.at(2)*a_.at(3)*a_.at(4)*a_.at(5)*a_.at(6)*a_.at(7)*in;
    //std::cout << "End_pos: " << "\n" << "x: " << out(0) << "\n" << "y: " << out(1) << "\n" << "z: " << out(2) << "\n";
    return (Point){out[0], out[1], out[2]};
}

rrt::rrt(joint_angles start_point, joint_angles goal_point, rrt_params params):kdtree(flann::KDTreeIndexParams()) {
    // collision
    initialize_world();
    // informed rrt
    L.setZero();
    if(params.goal_joint) {
        calculateC(goal_point);
    }

    // robot params
    a << 0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0;
    d << 0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107;
    alpha << 0, -M_PI/2, M_PI/2, M_PI/2, -M_PI/2, M_PI/2, M_PI/2, 0;
    goal_p = get_end_effector(goal_point);

    // apply constraints and rrt params
    this->goal_point = goal_point;
    this->start_point = start_point;
    this->params = params;

    // init start node
    start_node = new rrt_node(this->start_point, get_end_effector(this->start_point), params);

    // init kd-tree for sorting
    kdtree = flann::Index<flann::L2_Simple<float>>(
            flann::KDTreeIndexParams());
    std::vector<float> data(6);
    kdtree.buildIndex(
            flann::Matrix<float>(start_node->get_angles_flann()->data(), 1, 6));

    // start list of all nodes
    all_nodes.push_back(start_node);
}

// expand routine
tuple<bool, array<Point, 2>> rrt::expand() {
    rrt_node* nearest_node;
    rrt_node* new_node;
    joint_angles sample_point;
    joint_angles stepped_point;
    bool n_feasible = true;
    while(n_feasible) {
        if(params.goal_joint) {
            bool not_feasible = true;
            joint_angles sampled;
            while (not_feasible) {
                sampled = sample_intelligent();
                for (int i = 0; i < sampled.size(); i++) {
                    if ((sampled[i] < params.joint_ranges[i][0] || sampled[i] > params.joint_ranges[i][1])) {
                        not_feasible = true;
                        break;
                    } else {
                        not_feasible = false;
                    }
                }
            }
            for (int i = 0; i < sampled.size(); i++) {
                sample_point[i] = (float) sampled[i];
            }
        }
        else if(goal_found){
            bool not_feasible = true;
            joint_angles sampled;
            while (not_feasible) {
                sampled = sample_ellipsoid();
                for (int i = 0; i < sampled.size(); i++) {
                    if ((sampled[i] < params.joint_ranges[i][0] || sampled[i] > params.joint_ranges[i][1])) {
                        not_feasible = true;
                        break;
                    } else {
                        not_feasible = false;
                    }
                }
            }
            for (int i = 0; i < sampled.size(); i++) {
                sample_point[i] = (float) sampled[i];
            }
        }
        else {
            auto sampled = random_point(params.joint_ranges);
            for (int i = 0; i < sampled.size(); i++) {
                sample_point[i] = (float) sampled[i];
            }
        }
        nearest_node = findNearestNode(sample_point);
        stepped_point = step_forward(nearest_node->get_angles(), sample_point, params.step_size);
        for(int i = 0; i<stepped_point.size();i++){
            if(stepped_point[i]>params.joint_ranges[i][0] && stepped_point[i]<params.joint_ranges[i][1])
            {
                n_feasible = false;
            }
            else{
                n_feasible = true;
                break;
            }
        }
        if(!n_feasible){
            if(check_collision(stepped_point)){
                n_feasible=true;
            }
        }
    }
    // TODO: Collision check, feasibility check
    new_node = new rrt_node(stepped_point, get_end_effector(stepped_point), params, nearest_node);

    // find best parent
    auto near = findNearNodes(stepped_point);
    for(rrt_node* no:near){
        if(no!=new_node) {
            float temp2;
            temp2 = euclidean_dist_joint(no->get_angles(), new_node->get_angles());
            float temp = no->cost + temp2;
            if (new_node->cost > temp) {
                new_node->set_parent(no, temp, temp2);
            }
        }
    }

    // check if new node is better parent for other neighbor nodes
    for(rrt_node* no:near){
        if(no!=new_node) {
            float temp2;
            temp2 = euclidean_dist_joint(no->get_angles(), new_node->get_angles());
            float temp = new_node->cost + temp2;
            if (no->cost > temp) {
                no->set_parent(new_node, temp, temp2);
            }
        }
    }

    // condition kd-tree
    kdtree.addPoints(flann::Matrix<float>(new_node->get_angles_flann()->data(), 1, 6));
    //nodemap.insert(std::pair<Point, rrt_node*>(new_node->get_pos(), new_node));
    all_nodes.push_back(new_node);
    num_nodes++;

    // eval new node
    float dist;
    float dist_orient = 0;
    if(params.goal_joint){
        dist = euclidean_dist_sqrd_joint(new_node->get_angles(), goal_point);
        if (dist < min_dist || min_dist == -1){
            min_dist = dist;
        }
    }
    else{
        dist = euclidean_dist_sqrd(new_node->get_position(), goal_p);
        auto norm = get_end_effector_normal(new_node->get_angles());
        float ang_cost = 0;
        for(int i = 0; i<norm.size(); i++){
            ang_cost+=norm[i]*goal_normal[i];
        }
        if(ang_cost<=-1.0){
            dist_orient = M_PI;
        }
        else if(ang_cost>=1.0){
            dist_orient = 0;
        }
        else {
            dist_orient = abs(acos(ang_cost));
        }
        if ((dist_orient < min_dist_orient && dist < min_dist )|| (min_dist_orient == -1  && min_dist == -1)){
            min_dist_orient = dist_orient;
            min_dist = dist;
        }

    }
    if(dist<0.005 && dist_orient < 0.2){
        //goal_found = true;
        if(nodesmark_goal_found == 0) {
            goal_point_found = new_node->get_angles();
            //calculateC(goal_point_found);
            nodesmark_goal_found = num_nodes;
        }
        if(goal_node){
            if(new_node->cost < goal_node->cost){
                min_dist_orient = dist_orient;
                min_dist = dist;
                goal_node = new_node;
            }
        }
        else{
            goal_node = new_node;
        }

        if(num_nodes>=(long long) params.num_nodes_extra + nodesmark_goal_found){
            new_node = new rrt_node(goal_point, get_end_effector(goal_point), params, goal_node);
            goal_node = new_node;
            ROS_INFO("Goal pos: x %f y %f z %f", goal_p[0], goal_p[1], goal_p[2]);
            ROS_INFO("Goal pos planned: x %f y %f z %f", goal_node->get_position()[0], goal_node->get_position()[1], goal_node->get_position()[2]);
            return make_tuple(true, (array<Point, 2>){new_node->get_parent()->get_position(), new_node->get_position()});
        }
        return make_tuple(false, (array<Point, 2>){new_node->get_parent()->get_position(), new_node->get_position()});
    }
    if(goal_node && num_nodes>=(long long) params.num_nodes_extra + nodesmark_goal_found){
        new_node = new rrt_node(goal_point, get_end_effector(goal_point), params, goal_node);
        goal_node = new_node;
        ROS_INFO("Goal pos: x %f y %f z %f", goal_p[0], goal_p[1], goal_p[2]);
        ROS_INFO("Goal pos planned: x %f y %f z %f", goal_node->get_position()[0], goal_node->get_position()[1], goal_node->get_position()[2]);
        return make_tuple(true, (array<Point, 2>){new_node->get_parent()->get_position(), new_node->get_position()});
    }
    // TODO repeat until new node in case of collision
    return make_tuple(false, (array<Point, 2>){new_node->get_parent()->get_position(), new_node->get_position()});

}


// informed rrt (from ellipsoid)
joint_angles rrt::sample_ellipsoid(){
    joint_angles ret;
    float c_max;
    if(goal_node) {
        c_max = goal_node->cost;
    }
    else{
        c_max = 2*c_opt;
    }
    float temp = sqrt(c_max*c_max - c_opt*c_opt)/2;

    L(0,0) = c_max/2;
    for(int i = 1; i<6; i++){
        L(i, i) = temp;
    }
    auto ball_sampled = sample_unit_ball();
    auto rand_ = C * L * ball_sampled + center;
    for(int i = 0; i<ret.size(); i++){
        ret[i] = rand_(i);
    }
    return ret;
}


// find closest node to given joint angles
rrt_node *rrt::findNearestNode(joint_angles& relative_to) {
    flann::Matrix<float> query;
    std::vector<float> data(6);
    point_to_flann(relative_to, data.data());
    query = flann::Matrix<float>(data.data(), 1,
                                 data.size());
    std::vector<int> i(query.rows);
    flann::Matrix<int> indices(new int[query.rows], query.rows, 1);
    std::vector<float> d(query.rows);
    flann::Matrix<float> dists(new float[query.rows], query.rows, 1);

    int n = kdtree.knnSearch(query, indices, dists, 1, flann::SearchParams());

    //Point point;
    //point = flann_to_point(kdtree.getPoint(indices[0][0]));
    return all_nodes[indices[0][0]];
}


// find close nodes to given joint angles within a radius
vector<rrt_node*> rrt::findNearNodes(joint_angles& relative_to) {
    flann::Matrix<float> query;
    std::vector<float> data(6);
    std::vector<rrt_node*> near_nodes;
    point_to_flann(relative_to, data.data());
    query = flann::Matrix<float>(data.data(), 1,
                                 data.size());
    std::vector<int> i(query.rows);
    std::vector< std::vector<int> > indices;
    std::vector< std::vector<float> > dists;
    std::vector<float> d(query.rows);

    int n = kdtree.radiusSearch(query, indices, dists, 0.09, flann::SearchParams());

    //Point point;
    //point = flann_to_point(kdtree.getPoint(indices[0][0]));
    near_nodes.reserve(indices[0].size());
    for(int i = 0; i<indices[0].size();i++){
        near_nodes.push_back(all_nodes[indices[0][i]]);
    }
    return near_nodes;
}


// eval goal-path
vector<tuple<Point, joint_angles>> rrt::return_goal_path() {
    rrt_node* current_node = goal_node;
    vector<tuple<Point, joint_angles>> goal_path;
    array<vector<float>, 6> joints_smooth;
    vector<Point> points;
    joint_angles temp;
    int counter = 0;
    // iterate over path
    while(current_node!= nullptr){
        counter++;
        current_node = current_node->get_parent();
    }
    array<vector<double>, 6> joints;
    vector<double> temp_(counter);
    joints[0] = temp_;
    joints[1] = temp_;
    joints[2] = temp_;
    joints[3] = temp_;
    joints[4] = temp_;
    joints[5] = temp_;
    vector<double> x_set(counter);
    array<vector<double>, 6> vels;
    vector<double> test_vect(counter);
    vels[0] = test_vect;
    vels[1] = test_vect;
    vels[2] = test_vect;
    vels[3] = test_vect;
    vels[4] = test_vect;
    vels[5] = test_vect;
    vector<double> T_set(counter);
    vector<double> Tf_set(counter);
    vector<double> tb_set(counter);
    counter = 0;
    current_node = goal_node;
    // again iterate
    // normalized to step size 1
    while(current_node!= nullptr) {
        T_set[counter] = 0;
        tb_set[counter] = 0;
        temp = current_node->get_angles();
        points.push_back(current_node->get_position());
        if(current_node == start_node){
            ROS_INFO("USING START NODE");
        }

        // generate relative time steps for each node by determining the distance (maybe use better metric)
        for(int i = 0; i<joints.size(); i++){
            joints[i].at(counter) = temp[i];
        }
        counter++;
        current_node = current_node->get_parent();
    }
    ROS_INFO("Goal path has %i nodes", counter);
    for(int j = 0; j < x_set.size()-1; j++) {
        T_set[j] = 0;
        tb_set[j] = 0;
        for(int i = 0; i<joints.size(); i++) {
            if(j == 0){
                T_set[j] = max(T_set[j], abs(joints[i].at(j+1) - joints[i].at(j)) / params.max_vels[i]);
            }
            else if (j == x_set.size()-1){
                T_set[j] = 0;
            }
            else {
                T_set[j] = max(T_set[j],
                               abs(joints[i].at(j + 1) - joints[i].at(j)) / params.max_vels[i]);
            }
        }
        for(int i = 0; i<joints.size(); i++) {
            if(j == 0){
                vels[i].at(j) = (joints[i].at(j+1)-joints[i].at(j))/T_set[j];
                tb_set[j] = max(tb_set[j],
                                abs(vels[i].at(j)) / params.max_accs[i]);
            }
            else if (j == x_set.size()-1){
                vels[i].at(j) = -joints[i].at(j)/T_set[j];
                tb_set[j] = max(tb_set[j],
                                abs(vels[i].at(j) - vels[i].at(j - 1)) / params.max_accs[i]);
            }
            else {
                vels[i].at(j) = (joints[i].at(j+1)-joints[i].at(j))/T_set[j];
                tb_set[j] = max(tb_set[j],
                                abs(vels[i].at(j) - vels[i].at(j - 1)) / params.max_accs[i]);
            }
        }
    }
    for(int i = 0; i < x_set.size()-1; i++){
        if(i == 0 && tb_set[i] > T_set[i]/2){
            double fi = sqrt(T_set[i]/tb_set[i]);
            for(int k = 0; k<joints.size(); k++){
                vels[k].at(i) = vels[k].at(i) * fi;
            }
            T_set[i] = T_set[i] / fi;
            tb_set[i] = T_set[i];
        }
        else if(i == x_set.size()-1 && tb_set[i] > T_set[i-1]/2){
            double fi = sqrt(T_set[i-1]/tb_set[i]);
            for(int k = 0; k<joints.size(); k++){
                vels[k].at(i) = vels[k].at(i) * fi;
            }
            T_set[i] = T_set[i] / fi;
            T_set[i - 1] = T_set[i - 1] / fi;
            tb_set[i] = min(T_set[i], T_set[i-1]);
        }
        else if(i != 0 && (tb_set[i] > T_set[i-1]/2 || tb_set[i] > T_set[i]/2)){
            double fi = sqrt(min(T_set[i], T_set[i-1])/tb_set[i]);
            for(int k = 0; k<joints.size(); k++){
                vels[k].at(i - 1) = vels[k].at(i - 1) * fi;
                vels[k].at(i) = vels[k].at(i) * fi;
            }
            T_set[i] = T_set[i] / fi;
            T_set[i - 1] = T_set[i - 1] / fi;
            tb_set[i] = min(T_set[i], T_set[i-1]);
        }
    }
    for(int j = 0; j < x_set.size() -  1; j++){
        for(int i = 0; i<joints.size(); i++) {
            if(j == 0){
                tb_set[j] = max(tb_set[j],
                                abs(vels[i].at(j)) / params.max_accs[i]);
            }
            else {
                tb_set[j] = max(tb_set[j],
                                abs(vels[i].at(j) - vels[i].at(j - 1)) / params.max_accs[i]);
            }
        }
    }
    Tf_set[0] = tb_set[0]/2;
    for(int i = 1; i < x_set.size(); i++){
        Tf_set[i] = Tf_set[i-1] + T_set[i-1];
    }

    double t_max = tb_set[0]/2 + tb_set[T_set.size()-1]/2;;
    for(int i = 0; i<T_set.size()-1; i++){
        t_max+=T_set[i];
    }
    float t_elapsed = 0;
    counter = 0;
    int i = 0;
    while(t_elapsed<=t_max+0.0009){
        if(t_elapsed>t_max){
            t_elapsed = t_max;
        }
        double temp = 10000;
        double temp2 = 1000;
        if(i<T_set.size()-1){
            temp = Tf_set[i+1];
            temp2 = tb_set[i+1];
        }
        /*ROS_INFO("Intervals [%f, %f], [%f, %f], time %f", Tf_set[i]-tb_set[i]/2,
                  Tf_set[i] + tb_set[i]/2,  Tf_set[i] + tb_set[i]/2, temp - temp2/2, t_elapsed);*/

        if(t_elapsed >= Tf_set[i]-tb_set[i]/2 && t_elapsed <= Tf_set[i] + tb_set[i]/2){

            for(int k = 0; k<joints_smooth.size(); k++) {
                if(i == 0){
                    joints_smooth[k].push_back(joints[k].at(i) +
                                               0.5 *(vels[k].at(i))/tb_set[i] * pow((t_elapsed - Tf_set[i] + tb_set[i] / 2), 2));
                }
                else {
                    joints_smooth[k].push_back(joints[k].at(i) + vels[k].at(i-1) * (t_elapsed - Tf_set[i]) +
                                               0.5 * (vels[k].at(i)- vels[k].at(i-1))/tb_set[i] * pow((t_elapsed - Tf_set[i] + tb_set[i] / 2), 2));
                }
            }
            t_elapsed+=0.001;
        }
        else if(t_elapsed >= Tf_set[i] + tb_set[i]/2 && t_elapsed <= temp - temp2/2){
            for(int k = 0; k<joints_smooth.size(); k++) {
                    joints_smooth[k].push_back(joints[k].at(i) + vels[k].at(i) * (t_elapsed - Tf_set[i]));
            }
            t_elapsed+=0.001;
        }
        else{
            i++;
        }

    }
    Point start  = get_end_effector((joint_angles){joints_smooth[0].at(joints_smooth[0].size()-1), joints_smooth[1].at(joints_smooth[0].size()-1),
                                                   joints_smooth[2].at(joints_smooth[0].size()-1), joints_smooth[3].at(joints_smooth[0].size()-1),
                                                   joints_smooth[4].at(joints_smooth[0].size()-1), joints_smooth[5].at(joints_smooth[0].size()-1)});
    ROS_INFO("Start without smoothing: %f, %f, %f", start_node->get_position()[0],start_node->get_position()[1],start_node->get_position()[2]);
    ROS_INFO("Start with smoothing: %f, %f, %f", start[0], start[1], start[2]);
    for(int i = 0; i<joints_smooth[0].size(); i++){
        goal_path.push_back(make_tuple(get_end_effector((joint_angles){joints_smooth[0].at(i), joints_smooth[1].at(i),
                                                                       joints_smooth[2].at(i), joints_smooth[3].at(i),
                                                                       joints_smooth[4].at(i), joints_smooth[5].at(i)}),
                                       (joint_angles){joints_smooth[0].at(i), joints_smooth[1].at(i),
                                                      joints_smooth[2].at(i), joints_smooth[3].at(i),
                                                      joints_smooth[4].at(i), joints_smooth[5].at(i)}));
    }
    return goal_path;
}

void rrt::calculateC(joint_angles gp) {
    Matrix<float, 6, 1> a1;
    Matrix<float, 1, 6> v1 = {1, 0, 0, 0, 0, 0};
    for (int i = 0; i < start_point.size(); i++) {
        a1(i) = gp[i] - start_point[i];
    }
    a1 = a1 / a1.norm();
    Matrix<float, 6, 6> M = a1 * v1;
    JacobiSVD<Matrix<float, 6, 6>> svd(M, ComputeFullU | ComputeFullV);
    Matrix<float, 6, 1> vec_temp = {1, 1, 1, 1, 1, svd.matrixU().determinant() * svd.matrixV().determinant()};
    Matrix<float, 6, 6> temp_ = vec_temp.asDiagonal();
    C = svd.matrixU() * temp_ * svd.matrixV().transpose();
    c_opt = euclidean_dist_joint(start_point, gp);
    for(int i = 0; i<gp.size(); i++){
        center(i) = (gp[i] + start_point[i])/2;
    }

}

// with given probability sample goal point (improve convergence of the algorithm)
joint_angles rrt::sample_intelligent(){
    // TODO Change sampling when goal is found to optimize path
    float p = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    if(p<0.1){
        return goal_point;
    }
    else{
        return random_point(params.joint_ranges);
    }
}

void rrt::initialize_world() {
    mConfig = new btDefaultCollisionConfiguration();
    mDispatcher = new btCollisionDispatcher(mConfig);
    mBroadphase = new btDbvtBroadphase();
    mColWorld = new btCollisionWorld(mDispatcher, mBroadphase, mConfig);
    auto cyl0 = add_cylinder(btVector3(0.05, 0.3/2, 0.05), 2);
    auto cyl1 = add_cylinder(btVector3(0.06, 0.316/2, 0.06), 1);
    auto cyl2 = add_cylinder(btVector3(0.05, 0.2/2, 0.05), 2);
    auto cyl3 = add_cylinder(btVector3(0.07, 0.385/2, 0.07),2);
    auto cyl4 = add_cylinder(btVector3(0.05, 0.2, 0.05), 2);
    auto sphere = add_sphere(0.1);
    btTransform tr;
    tr.setOrigin(btVector3(0.45, 0, 0.7));
    sphere->setWorldTransform(tr);
    mObjects.push_back(cyl0);
    mObjects.push_back(cyl1);
    mObjects.push_back(cyl2);
    mObjects.push_back(cyl3);
    mObjects.push_back(cyl4);
    mObjects.push_back(sphere);
}

btCollisionObject* rrt::add_cylinder(btVector3 vect, int orient) {
    btCylinderShape* cylShape;
    if(orient == 0){
        cylShape = new btCylinderShapeX(vect);
    }
    else if(orient == 1){
        cylShape = new btCylinderShape(vect);
    }
    else{
        cylShape = new btCylinderShapeZ(vect);
    }
    ROS_INFO("1 rad: %f", cylShape->getRadius());
    btCollisionShape* colShape = (btCollisionShape*)cylShape;
    btCollisionObject* colObj = new btCollisionObject;
    void* userPointer = 0;
    int userIndex = ob_count;
    ob_count++;
    colObj->setUserIndex(userIndex);
    colObj->setUserPointer(userPointer);
    colObj->setCollisionShape(colShape);
    mColWorld->addCollisionObject(colObj);
    return colObj;
}
btCollisionObject* rrt::add_sphere(btScalar radius) {
    btSphereShape* sphereShape = new btSphereShape(radius);
    btCollisionShape* colShape = (btCollisionShape*)sphereShape;
    btCollisionObject* colObj = new btCollisionObject;
    void* userPointer = 0;
    int userIndex = ob_count;
    ob_count++;
    colObj->setUserIndex(userIndex);
    colObj->setUserPointer(userPointer);
    colObj->setCollisionShape(colShape);
    mColWorld->addCollisionObject(colObj);
    return colObj;
}

bool rrt::check_collision(joint_angles angles) {
    // neglect first link
    btTransform tr;
    Matrix<float, 4, 4> joint0;
    Matrix<float, 4, 4> joint1;
    Matrix<float, 4, 4> joint2;
    Matrix<float, 4, 4> joint3;
    Matrix<float, 4, 4> joint4;
    Matrix<float, 4, 4> joint5;
    Matrix<float, 4, 4> joint6;
    Matrix<float, 4, 4> joint7; // end effector
    get_transformationmatrix2(angles[0], a(0), d(0), alpha(0), joint0);
    get_transformationmatrix2(angles[1], a(1), d(1), alpha(1), joint1);
    joint1 = joint0*joint1;
    Matrix<float, 4, 1> col_center_0 = joint1(seq(0, last), 3);
    btMatrix3x3 rot0;
    eig_to_bt(joint1, rot0);
    tr.setOrigin(btVector3(col_center_0[0], col_center_0[1], col_center_0[2]));
    tr.setBasis(rot0);
    mObjects.at(0)->setWorldTransform(tr);
    Matrix<float, 4, 1> in_1;
    in_1 << 0, -0.158, 0, 1;
    Matrix<float, 4, 1> col_center_1 = joint1*in_1;
    tr.setOrigin(btVector3(col_center_1[0], col_center_1[1], col_center_1[2]));
    tr.setBasis(rot0);
    mObjects.at(1)->setWorldTransform(tr);


    get_transformationmatrix2(angles[2], a(2), d(2), alpha(2), joint2);
    joint2 = joint1*joint2;
    get_transformationmatrix2(angles[3], a(3), d(3), alpha(3), joint3);
    joint3 = joint2*joint3;
    Matrix<float, 4, 1> col_center_2 = joint3(seq(0, last), 3);
    btMatrix3x3 rot2;
    eig_to_bt(joint3, rot2);
    tr.setOrigin(btVector3(col_center_2[0], col_center_2[1], col_center_2[2]));
    tr.setBasis(rot2);
    mObjects.at(2)->setWorldTransform(tr);

    get_transformationmatrix2(angles[4], a(4), d(4), alpha(4), joint4);
    joint4 = joint3*joint4;
    Matrix<float, 4, 1> in_3;
    in_3 << 0, 0, -0.192, 1;
    Matrix<float, 4, 1> col_center_3 = joint4*in_3;
    tr.setOrigin(btVector3(col_center_3[0], col_center_3[1], col_center_3[2]));
    tr.setBasis(rot2);
    mObjects.at(3)->setWorldTransform(tr);

    get_transformationmatrix2(angles[5], a(5), d(5), alpha(5), joint5);
    joint5 = joint4*joint5;
    get_transformationmatrix2(0, a(6), d(6), alpha(6), joint6);
    joint6 = joint5*joint6;
    Matrix<float, 4, 1> col_center_4 = joint6(seq(0, last), 3);
    btMatrix3x3 rot4;
    eig_to_bt(joint6, rot4);
    tr.setOrigin(btVector3(col_center_4[0], col_center_4[1], col_center_4[2]));
    tr.setBasis(rot4);
    mObjects.at(4)->setWorldTransform(tr);


    get_transformationmatrix2(0, a(7), d(7), alpha(7), joint7);
    joint7 = joint6*joint7;

    SimulationContactResultCallback cb;
    for(int i = 0; i<5; i++){
        mColWorld->contactPairTest(mObjects.at(i), mObjects.at(5), cb);
        if(cb.bCollision){
            return true;
        }

        /*ROS_INFO("Cyl %i has pos x: %f, y: %f, z: %f", i, mObjects.at(i)->getWorldTransform().getOrigin().getX(),
                 mObjects.at(i)->getWorldTransform().getOrigin().getY(),
                 mObjects.at(i)->getWorldTransform().getOrigin().getZ());*/
    }
    return false;
}

