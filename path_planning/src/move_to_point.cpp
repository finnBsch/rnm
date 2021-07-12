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
const float fac = 0.04;
const float fac2 = 0.04;
std::vector<float> max_vels = {2.175f*fac, 2.175f*fac, 2.175f*fac, 2.175f*fac, 2.61f*fac, 2.61f*fac, 2.61f*fac};
std::vector<float> max_accs = {15.0f*fac2, 7.5f*fac2, 10.0f*fac2, 12.5f*fac2, 15.0f*fac2, 20.0f*fac2, 20.0f*fac2};

void callback(const std_msgs::Float64MultiArray& msg) {
  //auto start = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/franka_state_controller/joint_states_desired",ros::Duration(10)));
  auto start = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",ros::Duration(10)));
  Ruckig<7> otg {0.001}; // Number DoFs; control cycle in [s]
  InputParameter<7> input; // Number DoFs
  input.current_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  input.current_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  input.target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  input.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  input.max_jerk = {10.0, 10.0, 10.0, 4.0, 4.0, 4.0, 4.0};
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
  output.trajectory.calculate<false, true>(input, (double)0.001);
  vector<array<double, 7>> goal_trajectory;
  trajectory_msgs::JointTrajectory traj_msg;
  size_t sample_count = std::ceil(output.trajectory.get_duration() / 0.001);
  array<double, 7> one_point;
  for (size_t sample = 0; sample <= sample_count; ++sample)
  {
    // always sample the end of the trajectory as well
    double t = std::min(output.trajectory.get_duration(), sample * 0.001);
    array<double,7> np;
    array<double,7> np_;
    array<double,7> np__;
    output.trajectory.at_time(t,np,np_,np__);
    for (size_t j = 0; j < 7; ++j)
    {
      one_point[j] = np[j];
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
  traj_pub = new ros::Publisher(nh.advertise<trajectory_msgs::JointTrajectory>("/trajectory_sim", 10));
  ros::Subscriber sub = nh.subscribe("/move_command", 10, callback);
  ros::spin();
}
