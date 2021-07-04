#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>
#include "Trajectory.h"
#include "Path.h"
#include "sensor_msgs/JointState.h"
#include <eigen3/Eigen/Dense>
#include <list>
#include "utility.h"
#include "trajectory_msgs/JointTrajectory.h"
using namespace Eigen;
using namespace std;
ros::Publisher* traj_pub;
const float fac = 0.04;
const float fac2 = 0.003;
std::vector<float> max_vels = {2.175f*fac, 2.175f*fac, 2.175f*fac, 2.175f*fac, 2.61f*fac, 2.61f*fac, 2.61f*fac};
std::vector<float> max_accs = {15.0f*fac2, 7.5f*fac2, 10.0f*fac2, 12.5f*fac2, 15.0f*fac2, 20.0f*fac2, 20.0f*fac2};

void callback(const std_msgs::Float64MultiArray& msg) {
    auto start = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/franka_state_controller/joint_states_desired",ros::Duration(10)));
    //auto start = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",ros::Duration(10)));
    list<VectorXd> waypoints;
    VectorXd waypoint(7);
    VectorXd max_acc(7);
    VectorXd max_vel(7);
    for(int j = 0; j < msg.data.size(); j++){
        waypoint[j] = start.position[j];
    }
    waypoints.push_back(waypoint);
    for(int j = 0; j < msg.data.size(); j++){
        waypoint[j] = msg.data[j];
    }
    waypoints.push_back(waypoint);
    for(int i = 0; i < 7; i++){
        max_acc[i] = max_accs[i];
        max_vel[i] = max_vels[i];
    }
    Trajectory trajectory(Path(waypoints, 0.01),max_vel, max_acc, 0.0005);
    trajectory.outputPhasePlaneTrajectory();
    vector<array<double, 7>> goal_trajectory;
    trajectory_msgs::JointTrajectory traj_msg;
    if(trajectory.isValid()) {
        size_t sample_count = std::ceil(trajectory.getDuration() / 0.001);
        array<double, 7> one_point;
        for (size_t sample = 0; sample <= sample_count; ++sample)
        {
            // always sample the end of the trajectory as well
            double t = std::min(trajectory.getDuration(), sample * 0.001);
            auto pos = trajectory.getPosition(t);
            for (size_t j = 0; j < 7; ++j)
            {
                one_point[j] = pos[j];
            }
            trajectory_msgs::JointTrajectoryPoint pt;
            pt.positions.assign(one_point.begin(), one_point.end());
            traj_msg.points.push_back(pt);
        }
        traj_pub->publish(traj_msg);
        ros::spinOnce();
    }
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "move_to_point_node");
    ros::NodeHandle  nh("~");
    traj_pub = new ros::Publisher(nh.advertise<trajectory_msgs::JointTrajectory>("/trajectory", 10));
    ros::Subscriber sub = nh.subscribe("/Scanner_node/move_command", 10, callback);
    ros::spin();
}
