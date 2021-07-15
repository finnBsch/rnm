#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>
#include <ruckig/ruckig.hpp>
#include "sensor_msgs/JointState.h"
#include <eigen3/Eigen/Dense>
#include <list>
#include "utility.h"
#include "trajectory_msgs/JointTrajectory.h"
using namespace ruckig;
using namespace Eigen;
using namespace std;
ros::Publisher* traj_pub;
const float fac = 0.07;
const float fac2 = 0.005;
std::vector<float> max_vels = {2.175f*fac, 2.175f*fac, 2.175f*fac, 2.175f*fac, 2.61f*fac, 2.61f*fac, 2.61f*fac};
std::vector<float> max_accs = {15.0f*fac2, 7.5f*fac2, 10.0f*fac2, 12.5f*fac2, 15.0f*fac2, 20.0f*fac2, 20.0f*fac2};

void callback(const std_msgs::Float64MultiArray& msg) {
  auto start = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/franka_state_controller/joint_states_desired",ros::Duration(10)));
  //auto start = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",ros::Duration(10)));
  Ruckig<7> otg {0.001}; // Number DoFs; control cycle in [s]
  InputParameter<7> input; // Number DoFs
  input.current_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  input.current_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  input.target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  input.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double fac3 = 0.005;
  input.max_jerk = {fac3, fac3,fac3,fac3,fac3,fac3,fac3};
  for(int j = 0; j < 7; j++){
    input.current_position[j] = start.position[j];
  }

  for(int j = 0; j < 7; j++){
    input.target_position[j] = msg.data[j];
  }
  for(int i = 0; i < 7; i++) {
    input.max_acceleration[i] = max_accs[i];
    input.max_velocity[i] = max_vels[i];
  }
  OutputParameter<7> output;
  vector<array<double, 7>> goal_trajectory;
  trajectory_msgs::JointTrajectory traj_msg;
  array<double, 7> one_point;
  for (size_t j = 0; j < 7; ++j)
  {
    one_point[j] = input.current_position[j];
  }
  trajectory_msgs::JointTrajectoryPoint pt;
  pt.positions.assign(one_point.begin(), one_point.end());
  traj_msg.points.push_back(pt);
  while (otg.update(input, output) == Result::Working) {
    auto& p = output.new_position;
    //std::cout << output.time << " " << p[0] << " " << p[1] << " " << p[2] << " " << p[3] << " " << p[4] << " " << p[5] << std::endl;

    input.current_position = output.new_position;
    input.current_velocity = output.new_velocity;
    input.current_acceleration = output.new_acceleration;
    array<double, 7> one_point;
    for (size_t j = 0; j < 7; ++j)
    {
      one_point[j] = input.current_position[j];
    }
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions.assign(one_point.begin(), one_point.end());
    traj_msg.points.push_back(pt);
  }
  traj_pub->publish(traj_msg);
  ros::spinOnce();
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "move_to_point_node");
  ros::NodeHandle  nh("~");
  traj_pub = new ros::Publisher(nh.advertise<trajectory_msgs::JointTrajectory>("/trajectory", 10));
  ros::Subscriber sub = nh.subscribe("/move_command", 10, callback);
  ros::spin();
}
