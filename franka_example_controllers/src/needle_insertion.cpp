//
// Created by tolga on 02.06.21.
//

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <stdlib.h>
#include <math.h>
#include <sensor_msgs/JointState.h>
#include "inverse_kinematics/unserService.h"
#include "forward_kin/get_endeffector.h"
#include <eigen3/Eigen/Dense>
#include "franka_example_controllers/needle_insertion_service.h"

using namespace Eigen;




class RobotArm
{

 private:
  ros::NodeHandle nh_;
  std::vector<std::string> joint_names_;
  unsigned int num_joints_;
  std::string command_topic_;
  long counter = 0;
  int amount_of_movements_ = 0;
  std::vector<double> init_position{};
  ros::Publisher command_pub;
  std::array<double,7> finalJointAngles;
  std::vector<double> delta_angle;
  int scaling_ = 100;
  std::array<std::array<double, 7>, 100> received_joint_angles;
  ros::ServiceClient client;
  inverse_kinematics::unserService srv;
  sensor_msgs::JointState joint_state_msg;

 public:

  MatrixXd getLinePoints(VectorXd o1, VectorXd o2){
    VectorXd p1(3);
    VectorXd p2(3);
    VectorXd line(3);
    VectorXd scaledLine(3);

    p1 << o1(0),o1(1),o1(2);  //Endpunkt (x,y,z) aus Pathplanning
    p2 << o2(0),o2(1),o2(2);  //Zielpunkt (x,y,z)
    line=p2-p1;      //Strecke zwischen p1 und p2
    scaledLine=line/scaling_; //Skallierte Länge der Strecke
    MatrixXd LinePoints(3,scaling_);
    for(int i=0;i<scaling_;i++){
      //Speichert alle Punkte (x,y,z) in Matrix Linepoints (3xi) ab
      LinePoints(0,i)=    p1(0)+scaledLine(0)*(i+1);
      LinePoints(1,i)=    p1(1)+scaledLine(1)*(i+1);
      LinePoints(2,i)=    p1(2)+scaledLine(2)*(i+1);
    }

    return LinePoints;
  }


  // Initialize
  RobotArm(ros::NodeHandle nh, std::string command_topic): nh_(nh), num_joints_(7), command_topic_(command_topic_),init_position(7),delta_angle(7)
  {
    client = nh.serviceClient<inverse_kinematics::unserService>("/inverse_kinematics_node/unserService");
  }

  void sendStepCommand()
  {
    std::vector<double> goal_position;
    // calculate new joint angles
    // here it is just a sine wave on the initial joint angles
    double incrementalCounter =counter/10.;

    bool simYes = true;
    if(simYes){

      for (int i = 0; i < 7; ++i) {
        if (i == 4) {
          goal_position.push_back(finalJointAngles[i]);
        } else {
          goal_position.push_back(finalJointAngles[i]);
        }

      }
    } else{


      for (int i = 0; i < 7; i++) {
        delta_angle[i] = finalJointAngles[i] - init_position[i];
      }
      for (int i = 0; i < 7; i++) {
        delta_angle[i] = delta_angle[i] * incrementalCounter;

      }
      ROS_INFO("DELTA 0 %f", delta_angle[0] + init_position[0]);
      for (int i = 0; i < 7; ++i) {
        if (i == 4) {
          goal_position.push_back(init_position[i] + delta_angle[i]);
        } else {
          goal_position.push_back(init_position[i] + delta_angle[i]);
        }
        // ROS_INFO("Delta_angle %f",delta_angle[i]+init_position[i]);
        ROS_INFO("-------------------");
      }


    }

    counter++;


    // create message and publish it
    std_msgs::Float64MultiArray msg;
    msg.data.clear();
    msg.data.insert(msg.data.end(), goal_position.begin(), goal_position.end());
    command_pub.publish(msg);

    if(incrementalCounter>1) {
      for (int i = 0; i < 7; i++) {
        ROS_INFO("finalJointAngle %f", finalJointAngles[i]);
      }
      amount_of_movements_++;
      if (amount_of_movements_ == scaling_) {
        ros::shutdown();
      } else {
        ROS_INFO("Amount of Movement %i", amount_of_movements_);


        counter = 0;
        finalJointAngles = received_joint_angles[amount_of_movements_];
      }

    }

  }
  bool needle_insertion(inverse_kinematics::unserService::Request &req,
                        inverse_kinematics::unserService::Response &res) {

    if (command_topic_.find("sim") != std::string::npos) {
      joint_state_msg  = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",ros::Duration(10)));
    }
    else {
      joint_state_msg  = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/franka_state_controller/joint_states_desired",ros::Duration(10)));
    }

    for (size_t i = 0; i < 7; ++i) {
      init_position[i] = joint_state_msg.position[i];
    }

    //TODO request path planing points
    //FOR TESTING WITHOUT SERVICE
   /*
    //o1 << 0.417488, 0.0473712, 0.251279, 0.0, 3.14, 0.0;
    o1 << 0.617488, 0.0473712, 0.251279, 0.0, 3.14, 0.0;
    o2 << 0.617488, 0.2473712, 0.251279, 0.0, 3.14, 0.0;
    //o2 << 0.417488, 0.2473712, 0.251279, 0.0, 3.14, 0.0;
    //o2 << 0.417488, 0.0473712, 0.251279, 0.0, 3.14, 0.0;
    //o2 << 0.436342, 0.109329, 0.103379, 0.0, 3.14, 0.0; PUNKT VON NIKLAS*/

    VectorXd o1(6);
    VectorXd o2(6);
        o1 << req.start_point[0], req.start_point[1], req.start_point[2], req.start_point[3], req.start_point[4], req.start_point[5];
        o2 << req.end_point[0], req.end_point[1], req.end_point[2],req.end_point[3],req.end_point[4], req.end_point[5];

    MatrixXd points(3,scaling_);
    points = getLinePoints(o1, o2);

    for (int i = 0; i<points.rows();i++){
      for(int j = 0; j<points.cols();j++){
        ROS_INFO(" Endeffektor Position %f", points(i,j));
      }
      ROS_INFO("----------------------");
    }

    command_pub = nh_.advertise<std_msgs::Float64MultiArray>(command_topic_, 1);

    //give inverse kinematics the current joint angles of the robot
    srv.request.initial_joint_angles = {init_position[0],
                                        init_position[1],
                                        init_position[2],
                                        init_position[3],
                                        init_position[4],
                                        init_position[5],
                                        init_position[6]};

    for(int i = 0; i<scaling_;i++) {


      if(i>=1) {

        srv.request.initial_joint_angles = {received_joint_angles[i - 1][0],
                                            received_joint_angles[i - 1][1],
                                            received_joint_angles[i - 1][2],
                                            received_joint_angles[i - 1][3],
                                            received_joint_angles[i - 1][4],
                                            received_joint_angles[i - 1][5],
                                            received_joint_angles[i - 1][6]};
      }
      srv.request.desired_endeffector = {points(0, i), points(1, i), points(2, i), o1(3), o1(4), o1(5)};

      auto a = client.call(srv);
      if (a)
      {
        ROS_INFO("Connection Successful");
      }
      else
      {
        ROS_ERROR("Failed to call service inverse_kinematics");
        exit; //TODO find better solution
      }

      received_joint_angles[i] = {srv.response.ik_jointAngles[0],
                                  srv.response.ik_jointAngles[1],
                                  srv.response.ik_jointAngles[2],
                                  srv.response.ik_jointAngles[3],
                                  srv.response.ik_jointAngles[4],
                                  srv.response.ik_jointAngles[5],
                                  srv.response.ik_jointAngles[6]};
    }

    for (int i=0; i<scaling_;i++){
      for (int j =0;j<7; j++){
        ROS_INFO("Joint Angles %f", received_joint_angles[i][j]);
      }
      ROS_INFO("--------------------------");
    }

    finalJointAngles = received_joint_angles[0];
    sendStepCommand();
  }
};

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "simple_trajectory_player");
  ros::NodeHandle nh("~");


  // Amount of movement in each joint
  std::string command_topic = "/joint_position_example_controller_sim/joint_command";
  ros::param::get("~command_topic", command_topic);
  ROS_INFO_STREAM("command_topic: " << command_topic);

  // create RobotArm object
  RobotArm arm(nh,command_topic);
  ros::ServiceServer service = nh.advertiseService("unserService",&RobotArm::needle_insertion,&arm);



  // loop infinitely with a fixed frequency and send our commands
  ros::Rate loop_rate(1000);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
