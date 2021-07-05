// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>
#include <mutex>
#include <queue>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace franka_example_controllers {

class JointPositionExampleController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::PositionJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  ros::Duration elapsed_time_;
  ros::Duration last_exec;
  std::array<double, 7> initial_pose_{};
  int index = 0;

  ros::Subscriber command_sub_;
  ros::Subscriber trajectory_sub_;
  std::vector<double> command_;
  std::queue<std::vector<std::vector<double>>> traj_;
  std::vector<std::vector<double>> current_traj_;
  void setTrajCallback(const trajectory_msgs::JointTrajectoryConstPtr& msg);
  void setCommandCallback(const std_msgs::Float64MultiArrayConstPtr &msg);
  std::mutex mutex;
};

}  // namespace franka_example_controllers
