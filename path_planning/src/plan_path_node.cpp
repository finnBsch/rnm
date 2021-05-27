//
// Created by finn on 27.05.21.
//
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>
#include "rrt.h"
#include <chrono>
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
    Point start = {0, 0, 0};
    Point goal = {2, 2, 2};
    array<float, 2> range = {-2, 5};
    rrt_params params_ = {0.03, range, range, range, 0.5};
    rrt* tree = new rrt(start, goal, params_);
    bool not_found = true;
    float f = 0.0;
    visualization_msgs::Marker line_list;

    line_list.header.frame_id = "/world";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "plan_path_node";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;

    line_list.id = 2;

    line_list.type = visualization_msgs::Marker::LINE_LIST;


    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_list.scale.x = 0.02;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    auto start_time = std::chrono::high_resolution_clock::now();
    auto finish = start_time;
    while (not_found && ros::ok())
    {
        auto return_expand = tree->expand();
        not_found = !(get<0>(return_expand));

        geometry_msgs::Point p;
        auto new_line = get<1>(return_expand);
        p.x = new_line.at(0).at(0);
        p.y = new_line.at(0).at(1);
        p.z = new_line.at(0).at(2);
        line_list.points.push_back(p);
        p.x = new_line.at(1).at(0);
        p.y = new_line.at(1).at(1);
        p.z = new_line.at(1).at(2);
        line_list.points.push_back(p);
        //ros::spinOnce();
       // r.sleep();
        finish = std::chrono::high_resolution_clock::now();
        if((finish-start_time)/std::chrono::milliseconds(1)>1000/30){
            marker_pub.publish(line_list);
            start_time = finish;
        }
    }
}