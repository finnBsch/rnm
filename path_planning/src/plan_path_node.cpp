//
// Created by finn on 27.05.21.
//
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>
#include "rrt.h"
#include <chrono>
#include "trajectory_msgs/JointTrajectory.h"
#include "forward_kin/get_endeffector.h"
#include "inverse_kinematics/unserService.h"

/* good routes:
 * - [0.56, 0.3, 0.5] -> [0.12, 0.05, 0.4]*/
using namespace std::chrono;

int main(int argc, char **argv)
{
    bool sim = false;
    array<array<float, 2>, 6> joint_ranges;
    array<float, 2> range = {-2.8973, 2.8973};
    joint_ranges[0] = range;
    range = {-1.7628, 1.7628};
    joint_ranges[1] = range;
    range = {-M_PI/2, M_PI/2};
    joint_ranges[2] = range;
    range = {-3.0718, -0.0698};
    joint_ranges[3] = range;
    range = {-2.8973, 2.8973};
    joint_ranges[4] = range;
    range = {-0.0175, 3.7525};
    joint_ranges[5] = range;
    ros::init(argc, argv, "plan_path_node");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Publisher pub_accs = n.advertise<std_msgs::Float64MultiArray>("accs", 10);
    std_msgs::Float64MultiArray acc_msg;
    ros::Publisher traj_pub;
    forward_kin::get_endeffector srv;
    sensor_msgs::JointState joint_state_msg;
    ros::param::get("~sim", sim);
    if(sim){
        joint_state_msg = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",ros::Duration(10)));
        traj_pub = n.advertise<trajectory_msgs::JointTrajectory>("trajectory_sim", 10);
    }
    else {
        traj_pub = n.advertise<trajectory_msgs::JointTrajectory>("trajectory", 10);
        joint_state_msg = *(ros::topic::waitForMessage<sensor_msgs::JointState>(
                "/franka_state_controller/joint_states_desired", ros::Duration(10)));
    }
    inverse_kinematics::unserService srv_inv;
    ros::ServiceClient client = n.serviceClient<forward_kin::get_endeffector>("forward_kin_node/get_endeffector");
    ros::ServiceClient client_inv = n.serviceClient<inverse_kinematics::unserService>("inverse_kinematics_node/unserService");
    std_msgs::Float64MultiArray goal_point_ = *(ros::topic::waitForMessage<std_msgs::Float64MultiArray>("/goal_p",ros::Duration(10)));
    boost::array<double, 6> des_ef = {goal_point_.data[0], goal_point_.data[1], goal_point_.data[2], 0, 0, M_PI};
    boost::array<double, 7> arr = {joint_state_msg.position[0], joint_state_msg.position[1],joint_state_msg.position[2],
                                   joint_state_msg.position[3], joint_state_msg.position[4],joint_state_msg.position[5],
                                   joint_state_msg.position[6]};
    srv_inv.request.initial_joint_angles = arr;
    srv_inv.request.desired_endeffector = des_ef;
    srv.request.joint_angles = arr;
    auto a = client.call(srv);
    if (a)
    {
    }
    else
    {
        ROS_ERROR("Failed to call service forward_kin");
        return 1;
    }
    a = client_inv.call(srv_inv);
    if (a)
    {

    }
    else
    {
        ROS_ERROR("Failed to call service inverse_kinematics");
        //return 1;
    }
    bool not_feasible = true;
    bool succ = true;
    boost::array<double, 7> angles;
    while(not_feasible) {
        angles = srv_inv.response.ik_jointAngles;
        for(int i = 0; i < 6; i++){
            angles[i] = wrapMinMax(angles[i], -M_PI, M_PI);
        }
        for (int i = 0; i < 6; i++) {
            if ((angles[i] < joint_ranges[i][0] ||
                 angles[i] > joint_ranges[i][1] || !succ)) {
                //ROS_ERROR("GOAL POINT NOT FEASIBLE (out of range)");

                not_feasible = true;
                break;
            }
            else{
                not_feasible = false;
            }
        }
        if(not_feasible) {
            for(int j = 0; j <6; j++) {
                float p = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
                if(p<0.3) {
                    arr[j] = arr[j] * 1.1;
                }
                else if(p<0.5){
                    arr[j] = arr[j] *(-1);
                }
                else{
                    float LO = joint_ranges[j][0];
                    float HI = joint_ranges[j][1];
                    arr[j] = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
                }
            }
            srv_inv.request.initial_joint_angles = arr;
            succ = client_inv.call(srv_inv);
        }
    }
    //Point start = {(float)srv.response.end_effector_pos[0], (float)srv.response.end_effector_pos[1], (float)srv.response.end_effector_pos[2]};
    joint_angles start = {static_cast<float>(joint_state_msg.position[0]), static_cast<float>(joint_state_msg.position[1]),static_cast<float>(joint_state_msg.position[2]),
                          static_cast<float>(joint_state_msg.position[3]), static_cast<float>(joint_state_msg.position[4]),static_cast<float>(joint_state_msg.position[5])};

    float a_ = 2.7;
    float b_ = 1.2;
    float c_ = 0;
    float d_ = -0.7;
    float e_ = 0;
    float f_ = 0;
    float step_size = 0.01;
    bool goal_joint = true;
    int num_nodes_extra = 5000;
    /*ros::param::get("~ss", step_size);
    ros::param::get("~gj", goal_joint);
    ros::param::get("~nn", num_nodes_extra);
    ros::param::get("~a", a_);
    ros::param::get("~b", b_);
    ros::param::get("~c", c_);
    ros::param::get("~d", d_);
    ros::param::get("~e", e_);
    ros::param::get("~f", f_);*/
    joint_angles goal = {static_cast<float>(angles[0]), static_cast<float>(angles[1]), static_cast<float>(angles[2]),
                         static_cast<float>(angles[3]), static_cast<float>(angles[4]), static_cast<float>(angles[5])};
    ROS_INFO("Starting to find path to joint angles %f, %f, %f, %f, %f, %f",goal[0], goal[1], goal[2], goal[3], goal[4], goal[5]);

    float fac = 0.05;
    float fac2 = 0.05;
    std::vector<float> max_vels = {2.175f*fac, 2.175f*fac, 2.175f*fac, 2.175f*fac, 2.61f*fac, 2.61f*fac, 2.61f*fac};
    std::vector<float> max_accs = {15.0f*fac2, 7.5f*fac2, 10.0f*fac2, 12.5f*fac2, 15.0f*fac2, 20.0f*fac2, 20.0f*fac2};
    rrt_params params_ = {step_size, joint_ranges, goal_joint, num_nodes_extra, max_vels, max_accs};
    bool not_found = true;
    float f = 0.0;
    visualization_msgs::Marker sphere, goal_lines, lines_, rectangle;
    vector<visualization_msgs::Marker> line_list;
    line_list.push_back(sphere);
    line_list.push_back(goal_lines);
    line_list.push_back(lines_);
    line_list.push_back(rectangle);
    line_list[2].header.frame_id = "/world"; // TODO Find correct frame
    line_list[2].header.stamp = ros::Time::now();
    line_list[2].ns = "plan_path_node";
    line_list[2].action = visualization_msgs::Marker::ADD;
    line_list[2].pose.orientation.w = 1.0;

    line_list[2].id = 3;

    line_list[2].type = visualization_msgs::Marker::LINE_LIST;


    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_list[2].scale.x = 0.001;

    // Line list is red
    line_list[2].color.r = 1.0;
    line_list[2].color.a = 1.0;


    line_list[3].header.frame_id = "/world"; // TODO Find correct frame
    line_list[3].header.stamp = ros::Time::now();
    line_list[3].ns = "plan_path_node";
    line_list[3].action = visualization_msgs::Marker::ADD;
    line_list[3].pose.orientation.w = 1.0;

    line_list[3].id = 4;

    line_list[3].type = visualization_msgs::Marker::CUBE;

    line_list[3].color.g = 1.0;
    line_list[3].color.a = 1;
    line_list[3].scale.x = 0.5;
    line_list[3].scale.y = 1;
    line_list[3].scale.z = 0.3;
    line_list[3].pose.position.x = 0.35;
    line_list[3].pose.position.y = 0;
    line_list[3].pose.position.z = 0.1;



    line_list[0].header.frame_id = "/world"; // TODO Find correct frame
    line_list[0].header.stamp = ros::Time::now();
    line_list[0].ns = "plan_path_node";
    line_list[0].action = visualization_msgs::Marker::ADD;
    line_list[0].pose.orientation.w = 1.0;

    line_list[0].id = 2;

    line_list[0].type = visualization_msgs::Marker::SPHERE;


    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width

    // Line list is red
    line_list[0].color.r = 1.0;
    line_list[0].color.a = 1.0;
    line_list[0].pose.position.x = 0.35;
    line_list[0].pose.position.y = 0.2;
    line_list[0].pose.position.z = 0.5;
    line_list[0].scale.x = 0.15;
    line_list[0].scale.y = 0.15;
    line_list[0].scale.z = 0.15;
    marker_pub.publish(line_list[1]);
    marker_pub.publish(line_list[0]);
    marker_pub.publish(line_list[3]);
    auto start_time = std::chrono::high_resolution_clock::now();
    auto start_time_total = start_time;
    auto finish = start_time;
    rrt* tree = new rrt(start, goal, params_, &client_inv, srv_inv);
    while (not_found && ros::ok())
    {
        auto return_expand = tree->expand();
        not_found = !(get<0>(return_expand));

        /*geometry_msgs::Point p;
        auto new_line = get<1>(return_expand);
        p.x = new_line.at(0).at(0);
        p.y = new_line.at(0).at(1);
        p.z = new_line.at(0).at(2);
        line_list[2].points.push_back(p);
        p.x = new_line.at(1).at(0);
        p.y = new_line.at(1).at(1);
        p.z = new_line.at(1).at(2);
        line_list[2].points.push_back(p);*/
        finish = std::chrono::high_resolution_clock::now();
        if((finish-start_time)/std::chrono::milliseconds(1)>1000/10){
            //marker_pub.publish(line_list[2]);

            if(tree->goal_node!= nullptr){
                ROS_INFO("Number of nodes: %i and min dist %f and cost %f, %f", tree->num_nodes, tree->min_dist, tree->goal_node->cost, tree->min_dist_orient);
                ROS_INFO("Optimizing path for %i more nodes with cost %f", tree->num_nodes-(params_.num_nodes_extra + tree->nodesmark_goal_found), tree->goal_node->cost);

            }
            else{
                ROS_INFO("Number of nodes: %i and min dist %f, %f", tree->num_nodes, tree->min_dist, tree->min_dist_orient);
            }
            //marker_pub.publish(line_list[0]);
            start_time = finish;
        }

    }
    std::chrono::duration<double> duration = (finish - start_time_total);

    ROS_INFO_STREAM("Found Path in : " << duration.count());
    auto goal_path = tree->return_goal_path();
    auto planned_nodes = tree->return_planned_nodes();

    line_list[1].header.frame_id = "/world"; // TODO Find correct frame
    line_list[1].header.stamp = ros::Time::now();
    line_list[1].ns = "plan_path_node";
    line_list[1].action = visualization_msgs::Marker::ADD;
    line_list[1].pose.orientation.w = 1.0;

    line_list[1].id = 1;

    line_list[1].type = visualization_msgs::Marker::LINE_STRIP;


    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_list[1].scale.x = 0.002;

    // Line list is red
    line_list[1].color.b = 1.0;
    line_list[1].color.a = 1.0;
    geometry_msgs::Point p;
    trajectory_msgs::JointTrajectory traj_msg;
    trajectory_msgs::JointTrajectoryPoint pt;
    int i = 0;
    for(auto wp:goal_path.waypoints){
        pt.positions.clear();
        p.x = wp.pos3d[0];
        p.y = wp.pos3d[1];
        p.z = wp.pos3d[2];
        pt.positions.assign(wp.position.begin(), wp.position.end());
        pt.positions.push_back(static_cast<float>(joint_state_msg.position[6]));
        traj_msg.points.push_back(pt);
        if(i%100==0) {
            line_list[1].points.push_back(p);
        }
        i++;
    }
    ros::Rate loop_rate(1000);
    for(auto wp:goal_path.waypoints){
        acc_msg.data.clear();
        for(int i =0; i<wp.acceleration.size(); i++){
            acc_msg.data.push_back(wp.acceleration[i]);
        }
        pub_accs.publish(acc_msg);
        loop_rate.sleep();
    }
    marker_pub.publish(line_list[1]);
    marker_pub.publish(line_list[3]);
    traj_pub.publish(traj_msg);
    ros::spin();
}