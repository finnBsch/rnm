//
// Created by finn on 27.05.21.
//
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>
#include "rrt.h"
#include <chrono>
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
    Point start = {(float)srv.response.end_effector_pos[0], (float)srv.response.end_effector_pos[1], (float)srv.response.end_effector_pos[2]};
    Point goal = {0.2, 0.5, 0.3};
    array<float, 2> range_x = {-1, 1};
    array<float, 2> range_y = {-1, 1};
    array<float, 2> range_z = {0, 1.2};
    array<array<float, 2>,3> ranges;
    ranges[0]=(range_x);
    ranges[1]=(range_y);
    ranges[2]=(range_z);
    rrt_params params_ = {0.01,ranges};
    rrt* tree = new rrt(start, goal, params_);
    bool not_found = true;
    float f = 0.0;
    visualization_msgs::Marker lines, goal_lines;
    vector<visualization_msgs::Marker> line_list;
    line_list.push_back(lines);
    line_list.push_back(goal_lines);
    line_list[0].header.frame_id = "/world"; // TODO Find correct frame
    line_list[0].header.stamp = ros::Time::now();
    line_list[0].ns = "plan_path_node";
    line_list[0].action = visualization_msgs::Marker::ADD;
    line_list[0].pose.orientation.w = 1.0;

    line_list[0].id = 2;

    line_list[0].type = visualization_msgs::Marker::LINE_LIST;


    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_list[0].scale.x = 0.001;

    // Line list is red
    line_list[0].color.r = 1.0;
    line_list[0].color.a = 1.0;
    auto start_time = std::chrono::high_resolution_clock::now();
    auto start_time_total = start_time;
    auto finish = start_time;
    marker_pub.publish(line_list[1]);

    while (not_found && ros::ok())
    {
        auto return_expand = tree->expand();
        not_found = !(get<0>(return_expand));

        geometry_msgs::Point p;
        auto new_line = get<1>(return_expand);
        p.x = new_line.at(0).at(0);
        p.y = new_line.at(0).at(1);
        p.z = new_line.at(0).at(2);
        line_list[0].points.push_back(p);
        p.x = new_line.at(1).at(0);
        p.y = new_line.at(1).at(1);
        p.z = new_line.at(1).at(2);
        line_list[0].points.push_back(p);
        //ros::spinOnce();
       // r.sleep();
        finish = std::chrono::high_resolution_clock::now();
        if((finish-start_time)/std::chrono::milliseconds(1)>1000/10){
            ROS_INFO("Number of nodes: %i", line_list[0].points.size()/2);
            marker_pub.publish(line_list[0]);
            start_time = finish;
        }
    }
    std::chrono::duration<double> duration = (finish - start_time_total);
    ROS_INFO_STREAM("Found Path in : " << duration.count());
    marker_pub.publish(line_list[0]);
    auto goal_path = tree->return_goal_path();
    line_list[1].header.frame_id = "/world"; // TODO Find correct frame
    line_list[1].header.stamp = ros::Time::now();
    line_list[1].ns = "plan_path_node";
    line_list[1].action = visualization_msgs::Marker::ADD;
    line_list[1].pose.orientation.w = 1.0;

    line_list[1].id = 1;

    line_list[1].type = visualization_msgs::Marker::LINE_STRIP;


    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_list[1].scale.x = 0.009;

    // Line list is red
    line_list[1].color.b = 1.0;
    line_list[1].color.a = 1.0;
    geometry_msgs::Point p;
    for(auto point:goal_path){
        p.x = point[0];
        p.y = point[1];
        p.z = point[2];
        line_list[1].points.push_back(p);
    }
    ros::Rate r(1);
    while(ros::ok()){
        marker_pub.publish(line_list[1]);
        marker_pub.publish(line_list[0]);
        r.sleep();
    }
}