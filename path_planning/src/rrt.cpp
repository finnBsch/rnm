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
    L.setZero();
    if(params.goal_joint) {
        calculateC(goal_point);
    }
    a << 0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0;
    d << 0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107;
    alpha << 0, -M_PI/2, M_PI/2, M_PI/2, -M_PI/2, M_PI/2, M_PI/2, 0;
    goal_p = get_end_effector(goal_point);
    kdtree = flann::Index<flann::L2_Simple<float>>(
            flann::KDTreeIndexParams());
    // save params
    this->goal_point = goal_point;
    this->start_point = start_point;
    this->params = params;
    // init start node
    start_node = new rrt_node(this->start_point, get_end_effector(this->start_point), params);
    std::vector<float> data(6);
    //point_to_flann(start_node->get_angles_flann(), data.data());
    kdtree.buildIndex(
            flann::Matrix<float>(start_node->get_angles_flann()->data(), 1, 6));
    all_nodes.push_back(start_node);
    //kdtree.addPoints(flann::Matrix<float>(start_node->get_pos_flann()->data(), 1, 6));
    //nodemap.insert(pair<Point, rrt_node*>(start_node->get_pos(), start_node));

}
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
        //sample_point = random_point(params.joint_ranges);
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
    }
    // TODO: Collision check, feasibility check
    new_node = new rrt_node(stepped_point, get_end_effector(stepped_point), params, nearest_node);
    auto near = findNearNodes(stepped_point);
    for(rrt_node* no:near){
        float temp2;
        if(no == start_node){
            temp2 = euclidean_dist_joint(no->get_angles(), new_node->get_angles());
        }
        else{
            auto temp_ = no->cost_two_joints(no, new_node);
            temp2 = std::get<0>(temp_) + std::get<1>(temp_)*params.steercost;
        }
        float temp = no->cost + temp2;
        if(new_node->cost > temp){
            new_node->set_parent(no, temp, temp2);
        }
    }
    for(rrt_node* no:near){
        float temp2;
        if(no == start_node){
            temp2 = euclidean_dist_joint(no->get_angles(), new_node->get_angles());
        }
        else{
            auto temp_ = no->cost_two_joints(no, new_node);
            temp2 = std::get<0>(temp_) + std::get<1>(temp_)*params.steercost;
        }
        float temp = new_node->cost + temp2;
        if(no->cost > temp){
            no->set_parent(new_node, temp, temp2);
        }
    }
    kdtree.addPoints(flann::Matrix<float>(new_node->get_angles_flann()->data(), 1, 6));
    //nodemap.insert(std::pair<Point, rrt_node*>(new_node->get_pos(), new_node));
    all_nodes.push_back(new_node);
    num_nodes++;
    float dist;
    float dist_orient = 0;
    if(params.goal_joint){
        dist = euclidean_dist_sqrd_joint(new_node->get_angles(), goal_point);
    }
    else{
        dist = euclidean_dist_sqrd(new_node->get_position(), goal_p);
        auto norm = get_end_effector_normal(new_node->get_angles());
        float ang_cost = 0;
        for(int i = 0; i<norm.size(); i++){
            ang_cost+=norm[i]*goal_normal[i];
        }
        dist_orient = abs(acos(ang_cost));
    }
    if(params.goal_joint){
        if (dist < min_dist || min_dist == -1){
            min_dist = dist;
        }
    }
    else{
        if ((dist_orient < min_dist_orient && dist < min_dist )|| (min_dist_orient == -1  && min_dist == -1)){
            min_dist_orient = dist_orient;
            min_dist = dist;
        }
    }
    if(dist<0.005 && dist_orient < 0.2){
        //goal_found = true;
        if(nodesmark_goal_found == 0) {
            goal_point_found = new_node->get_angles();
            calculateC(goal_point_found);
            nodesmark_goal_found = num_nodes;
        }
        if(goal_node){
            if(new_node->cost < goal_node->cost){
                goal_node = new_node;
            }
        }
        else{
            goal_node = new_node;
        }

        if(num_nodes>=(long long) params.num_nodes_extra + nodesmark_goal_found){
            ROS_INFO("Goal pos: x %f y %f z %f", goal_p[0], goal_p[1], goal_p[2]);
            ROS_INFO("Goal pos planned: x %f y %f z %f", goal_node->get_position()[0], goal_node->get_position()[1], goal_node->get_position()[2]);
            return make_tuple(true, (array<Point, 2>){new_node->get_parent()->get_position(), new_node->get_position()});
        }
        return make_tuple(false, (array<Point, 2>){new_node->get_parent()->get_position(), new_node->get_position()});
    }
    if(goal_node && num_nodes>=(long long) params.num_nodes_extra + nodesmark_goal_found){
        ROS_INFO("Goal pos: x %f y %f z %f", goal_p[0], goal_p[1], goal_p[2]);
        ROS_INFO("Goal pos planned: x %f y %f z %f", goal_node->get_position()[0], goal_node->get_position()[1], goal_node->get_position()[2]);
        return make_tuple(true, (array<Point, 2>){new_node->get_parent()->get_position(), new_node->get_position()});
    }
    // TODO repeat until new node in case of collision
    return make_tuple(false, (array<Point, 2>){new_node->get_parent()->get_position(), new_node->get_position()});

}

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

    int n = kdtree.radiusSearch(query, indices, dists, 0.05, flann::SearchParams());

    //Point point;
    //point = flann_to_point(kdtree.getPoint(indices[0][0]));
    near_nodes.reserve(indices[0].size());
    for(int i = 0; i<indices[0].size();i++){
        near_nodes.push_back(all_nodes[indices[0][i]]);
    }
    return near_nodes;
}

vector<tuple<Point, joint_angles>> rrt::return_goal_path() {
    rrt_node* current_node = goal_node;
    vector<tuple<Point, joint_angles>> goal_path;
    array<vector<float>, 6> joints_smooth;
    vector<Point> points;
    joint_angles temp;

    int counter = 0;
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
    vector<double> dx_set(counter);
    counter = 0;
    current_node = goal_node;
    while(current_node!= nullptr) {
        if(counter==0) {
            x_set[counter] = 0;
        }
        dx_set[counter] = 0;
        temp = current_node->get_angles();
        points.push_back(current_node->get_position());
        if(current_node == start_node){
            ROS_INFO("USING START NODE");
        }
        for(int i = 0; i<joints.size(); i++){
            if(counter!=0){
                dx_set[counter] = max(dx_set[counter], abs(temp[i]-joints[i].at(counter-1))*5);
            }
            joints[i].at(counter) = temp[i];
        }
        x_set[counter] = x_set[counter-1] + dx_set[counter];
        counter++;
        current_node = current_node->get_parent();
    }
    vector<tk::spline> splines;

    for(int i = 0; i<joints.size(); i++){
        tk::spline s(x_set, joints[i], tk::spline::cspline,false,tk::spline::first_deriv, 0.0,
                     tk::spline::first_deriv, 0.0);
        splines.push_back(s);
    }
    float t_max = x_set[x_set.size()-1];
    for(int i = 0; i<joints.size(); i++){
        for(long long j = 0; j<joints[i].size()*100; j++){
            joints_smooth[i].push_back(splines[i]((float)j/(float)(joints[i].size()*100) * t_max));
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
