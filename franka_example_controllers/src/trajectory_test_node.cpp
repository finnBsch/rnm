//
// Created by finn on 18.05.21.
//
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <stdlib.h>
#include <math.h>
#include <sensor_msgs/JointState.h>

class RobotArm
{

 private:
  ros::NodeHandle nh_;
  std::vector<std::string> joint_names_;
  unsigned int num_joints_;
  std::vector<double> goal_pose_;
  double t_f_;
  std::vector<double> max_vels_ = {2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61};
  std::vector<std::vector<double>> trajectory_coeff;
  std::string command_topic_;
  long counter = 0;
  std::array<double, 7> init_position{};
  ros::Publisher command_pub;

 public:

  // Initialize
  RobotArm(ros::NodeHandle nh, std::vector<double> goal_pose, std::string command_topic, double t_f): t_f_(t_f), nh_(nh), num_joints_(7), goal_pose_(goal_pose), command_topic_(command_topic_)
  {
    sensor_msgs::JointState joint_state_msg;
    std::vector<double> max_acc_ = {15, 7.5, 10, 12.5, 15, 20, 20};
    if (command_topic.find("sim") != std::string::npos) {
      joint_state_msg  = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",ros::Duration(10)));
    }
    else {
      joint_state_msg  = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/franka_state_controller/joint_states_desired",ros::Duration(10)));
    }
    bool trajectory_feasible = false;
    while(!trajectory_feasible) {
      for (size_t i = 0; i < 7; ++i) {
        init_position[i] = joint_state_msg.position[i];
        std::vector<double> temp_coeff;
        double a = init_position[i];
        std::cout << "diff  " << goal_pose_[i] - init_position[i] << std::endl;
        temp_coeff.push_back(a);
        a = 10 / pow(t_f_, 3) * (goal_pose_[i] - init_position[i]);
        temp_coeff.push_back(a);
        a = -15 / pow(t_f_, 4) * (goal_pose_[i] - init_position[i]);
        temp_coeff.push_back(a);
        a = 6 / pow(t_f_, 5) * (goal_pose_[i] - init_position[i]);
        temp_coeff.push_back(a);
        trajectory_coeff.push_back(temp_coeff);
        double t_vel = t_f_/2;
        double max_vel = 3*temp_coeff[1]*pow(t_vel,2) + 4* temp_coeff[2]*pow(t_vel,3) + 5*temp_coeff[3]*pow(t_vel, 4);
        double t_acc = t_f_/4;
        double max_acc = 6*temp_coeff[1]*pow(t_acc,1) + 12* temp_coeff[2]*pow(t_acc,2) + 20*temp_coeff[3]*pow(t_acc, 3);
        if(abs(max_vel)>0.8*max_vels_[i] || abs(max_acc)>0.5*max_acc_[i]) {
          trajectory_feasible = false;
          trajectory_coeff.clear();
          break;
        }
        else{
          std::cout << "Joint " << i << " max vel " << max_vel << std::endl;
          trajectory_feasible = true;
        }
      }
      t_f_+=0.1;
    }
    ROS_INFO_STREAM("Executing trajectory in t : " << t_f_);
    command_pub = nh_.advertise<std_msgs::Float64MultiArray>(command_topic, 1);
  }

  bool sendStepCommand()
  {
    std::vector<double> goal_position;
    double joint_goal_angle;
    double t = 0;
    for (size_t i = 0; i < 7; ++i) {
      t = counter*0.001;
      joint_goal_angle = trajectory_coeff[i][0] + trajectory_coeff[i][1]*pow(t, 3) +
                         trajectory_coeff[i][2]*pow(t, 4) + trajectory_coeff[i][3]*pow(t, 5);

      goal_position.push_back(joint_goal_angle);
    }

    counter++;

    // create message and publish it
    std_msgs::Float64MultiArray msg;
    msg.data.clear();
    msg.data.insert(msg.data.end(), goal_position.begin(), goal_position.end());
    command_pub.publish(msg);
    if(t>=t_f_){
      for (size_t i = 0; i < 7; ++i) {
        std::cout << goal_position[i]*180/M_PI << std::endl;
      }
      return true;
    }
    return false;
  }
};

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "simple_trajectory_player");
  ros::NodeHandle nh;

  // Amount of movement in each joint
  double init_angle = 80*M_PI/180;
  double t_f =0.3;
  std::vector<double> goal_pose = {init_angle, init_angle, -init_angle, -init_angle, init_angle, init_angle  , init_angle};
  //std::vector<double> goal_pose = {0,0,-90*M_PI/180,-90*M_PI/180,90*M_PI/180,90*M_PI/180 , 0*90*M_PI/180};
  ros::param::get("~t_f", t_f);
  ros::param::get("~goal_pose", goal_pose);
  ROS_INFO_STREAM("goal_pose: " << goal_pose[0] << " " << goal_pose[1] << " " <<goal_pose[2]
                                << " " <<goal_pose[3] << " " << goal_pose[4] << " " <<goal_pose[5] << " " <<
                  goal_pose[6] << " " << " rad");

  // Amount of movement in each joint
  std::string command_topic = "/joint_position_example_controller_sim/joint_command";
  ros::param::get("~command_topic", command_topic);
  ROS_INFO_STREAM("command_topic: " << command_topic);

  // create RobotArm object
  RobotArm arm(nh, goal_pose,command_topic, t_f);

  // loop infinitely with a fixed frequency and send our commands
  ros::Rate loop_rate(1000);
  bool move_finished = false;
  while (ros::ok() && !move_finished)
  {
    move_finished=arm.sendStepCommand();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
