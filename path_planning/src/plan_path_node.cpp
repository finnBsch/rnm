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
using namespace std::chrono;
class RRT_Publisher
{
public:
    RRT_Publisher(ros::Publisher* pub, visualization_msgs::Marker* line_list)
    {
        this->line_list = line_list;
        rrtPublisher = pub;
    }
    void publishLine()
    {
        rrtPublisher->publish(*line_list);
    }
private:
    visualization_msgs::Marker* line_list;
    ros::Publisher* rrtPublisher;
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "plan_path_node");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Publisher traj_pub = n.advertise<trajectory_msgs::JointTrajectory>("trajectory", 10);
    forward_kin::get_endeffector srv;
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg  = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",ros::Duration(10)));
    ros::ServiceClient client = n.serviceClient<forward_kin::get_endeffector>("forward_kin_node/get_endeffector");
    boost::array<double, 7> arr = {joint_state_msg.position[0], joint_state_msg.position[1],joint_state_msg.position[2],
                                   joint_state_msg.position[3], joint_state_msg.position[4],joint_state_msg.position[5],
                                   joint_state_msg.position[6]};
    srv.request.joint_angles = arr;
    auto a = client.call(srv);
    if (a)
    {
        ROS_INFO("Endpos: %f", srv.response.end_effector_pos[0]);
    }
    else
    {
        ROS_ERROR("Failed to call service forward_kin");
        return 1;
    }
    //Point start = {(double)srv.response.end_effector_pos[0], (double)srv.response.end_effector_pos[1], (double)srv.response.end_effector_pos[2]};
    joint_angles start = {static_cast<double>(joint_state_msg.position[0]), static_cast<double>(joint_state_msg.position[1]),static_cast<double>(joint_state_msg.position[2]),
                   static_cast<double>(joint_state_msg.position[3]), static_cast<double>(joint_state_msg.position[4]),static_cast<double>(joint_state_msg.position[5])};

    double a_ = 0.1;
    double b_ = 1.2;
    double c_ = 0;
    double d_ = -0.7;
    double e_ = 0;
    double f_ = 0;
    double step_size = 0.03;
    bool goal_joint = true;
    int num_nodes_extra = 5000;
    ros::param::get("~ss", step_size);
    ros::param::get("~gj", goal_joint);
    ros::param::get("~nn", num_nodes_extra);
    ros::param::get("~a", a_);
    ros::param::get("~b", b_);
    ros::param::get("~c", c_);
    ros::param::get("~d", d_);
    ros::param::get("~e", e_);
    ros::param::get("~f", f_);
    joint_angles goal = {a_,b_,c_,d_,e_,f_};
    ROS_INFO("Starting to find path to joint angles %f, %f, %f, %f, %f, %f", a_, b_, c_, d_, e_, f_);

    array<array<double, 2>, 6> joint_ranges;
    array<double, 2> range = {-2.8973, 2.8973};
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
    double fac = 0.12;
    double fac2 = 0.02;
    std::vector<double> max_vels = {2.175f*fac, 2.175f*fac, 2.175f*fac, 2.175f*fac, 2.61f*fac, 2.61f*fac, 2.61f*fac};
    std::vector<double> max_accs = {15.0f*fac2, 7.5f*fac2, 10.0f*fac2, 12.5f*fac2, 15.0f*fac2, 20.0f*fac2, 20.0f*fac2};
    rrt_params params_ = {step_size, joint_ranges, goal_joint, num_nodes_extra, max_vels, max_accs};
    rrt* tree = new rrt(start, goal, params_);
    bool not_found = true;
    double f = 0.0;
    visualization_msgs::Marker lines, goal_lines, lines_, points_viz;
    vector<visualization_msgs::Marker> line_list;
    line_list.push_back(lines);
    line_list.push_back(goal_lines);
    line_list.push_back(lines_);
    line_list.push_back(points_viz);
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

    line_list[3].type = visualization_msgs::Marker::SPHERE_LIST;


    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width

    // Line list is red
    line_list[3].color.g = 1.0;
    line_list[3].color.a = 0.4;
    line_list[3].scale.x = 0.001;
    line_list[3].scale.y = 0.001;
    line_list[3].scale.z = 0.001;


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
    line_list[0].pose.position.x = 0.45;
    line_list[0].pose.position.y = 0;
    line_list[0].pose.position.z = 0.7;
    line_list[0].scale.x = 0.15;
    line_list[0].scale.y = 0.15;
    line_list[0].scale.z = 0.15;
    auto start_time = std::chrono::high_resolution_clock::now();
    auto start_time_total = start_time;
    auto finish = start_time;
    marker_pub.publish(line_list[1]);
    marker_pub.publish(line_list[0]);
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

    for(auto point_path_pair:goal_path){
        p.x = (std::get<0>(point_path_pair))[0];
        p.y = (std::get<0>(point_path_pair))[1];
        p.z = (std::get<0>(point_path_pair))[2];
        pt.positions.assign(std::get<1>(point_path_pair).begin(), std::get<1>(point_path_pair).end());
        traj_msg.points.push_back(pt);
        i++;
        line_list[1].points.push_back(p);
        line_list[3].points.push_back(p);
    }
    std::reverse(traj_msg.points.begin(),traj_msg.points.end());
    traj_pub.publish(traj_msg);
    ros::Rate r(1);
    while(ros::ok()){
        marker_pub.publish(line_list[1]);
        //marker_pub.publish(line_list[3]);
        marker_pub.publish(line_list[0]);
        r.sleep();
    }
}