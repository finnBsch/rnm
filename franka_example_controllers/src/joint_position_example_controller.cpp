// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_position_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>



namespace franka_example_controllers {

bool JointPositionExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointPositionExampleController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionExampleController: Wrong number of joint names, got "
                         << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionExampleController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  for (auto &joint_handle : position_joint_handles_) {
    command_.push_back(0.);
  }

  //command_sub_ = node_handle.subscribe<std_msgs::Float64MultiArray>(std::string("joint_command"), 1,
  //  &JointPositionExampleController::setCommandCallback, this,ros::TransportHints().tcpNoDelay());
  trajectory_sub_ = node_handle.subscribe<trajectory_msgs::JointTrajectory>(std::string("/trajectory"), 25,
                                                                            &JointPositionExampleController::setTrajCallback, this);
  return true;
}

void JointPositionExampleController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = position_joint_handles_[i].getPosition();
    command_[i] = initial_pose_[i];
  }
  elapsed_time_ = ros::Duration(0.0);
}

void JointPositionExampleController::update(const ros::Time&, const ros::Duration& period) {
  elapsed_time_ += period;
  mutex.lock();
  for (size_t i = 0; i < position_joint_handles_.size(); i++) {
    position_joint_handles_.at(i).setCommand(command_.at(i));
  }
  //if(elapsed_time_.toSec() - last_exec.toSec() >= 0.001 || true) {
  if (index < current_traj_.size()) {
    command_ = current_traj_[index];
    index++;
  }
  else if(!traj_.empty()){
    current_traj_ = traj_.front();
    traj_.pop();
    index = 0;
  }
  else {
    //command_ = current_traj_[index];
  }
//std::cout << "\n" << std::endl;
  mutex.unlock();
}

void JointPositionExampleController::setCommandCallback(const std_msgs::Float64MultiArrayConstPtr &msg) {
  mutex.lock();
  command_ = msg->data;
  mutex.unlock();
}
void JointPositionExampleController::setTrajCallback(const trajectory_msgs::JointTrajectoryConstPtr& msg) {
  mutex.lock();
  std::vector<std::vector<double>> one_traj;
  std::vector<double> com(7);
  for (int i = 0; i < msg->points.size(); i++) {
    for (int j = 0; j < 7; j++) {
      com[j] = msg->points[i].positions[j];
    }
    one_traj.push_back(com);
  }
  traj_.push(one_traj);
  ROS_INFO("Queue size %i", traj_.size());
  mutex.unlock();
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionExampleController,
                       controller_interface::ControllerBase)
