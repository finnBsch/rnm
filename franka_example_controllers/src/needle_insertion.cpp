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
#include "Trajectory.h"
#include "Path.h"
#include <list>
#include "trajectory_msgs/JointTrajectory.h"
#include "franka_example_controllers/needle_insertion_service.h"


using namespace std;
using namespace Eigen;




class RobotArm
{

private:
    float fac = 0.05;
    float fac2 = 0.003;
    std::vector<float> max_vels = {2.175f*fac, 2.175f*fac, 2.175f*fac, 2.175f*fac, 2.61f*fac, 2.61f*fac, 2.61f*fac};
    std::vector<float> max_accs = {15.0f*fac2, 7.5f*fac2, 10.0f*fac2, 12.5f*fac2, 15.0f*fac2, 20.0f*fac2, 20.0f*fac2};
    ros::NodeHandle nh_;
    std::vector<std::string> joint_names_;
    ros::Publisher* traj_pub;
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
    RobotArm(ros::NodeHandle nh): nh_(nh), num_joints_(7), init_position(7),delta_angle(7)
    {
        traj_pub = new ros::Publisher(nh_.advertise<trajectory_msgs::JointTrajectory>("/trajectory", 10));



        sensor_msgs::JointState joint_state_msg;

        joint_state_msg  = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/franka_state_controller/joint_states_desired",ros::Duration(10)));

        for (size_t i = 0; i < 7; ++i) {
            init_position[i] = joint_state_msg.position[i];
        }



    }

    void sendStepCommand()
    {
        std::vector<double> goal_position;
        // calculate new joint angles
        // here it is just a sine wave on the initial joint angles
        double incrementalCounter =counter/10000.;

        bool simYes = false;
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

    bool needle_insertion(franka_example_controllers::needle_insertion_service::Request &req,
                      franka_example_controllers::needle_insertion_service::Response &res) {
      inverse_kinematics::unserService srv;
      ros::ServiceClient client = nh_.serviceClient<inverse_kinematics::unserService>("/inverse_kinematics_node/unserService");

      //TODO request path planing points
      VectorXd o1(6);
      VectorXd o2(6);


      o1 << req.start_pos_endeffector[0], req.start_pos_endeffector[1], req.start_pos_endeffector[2], req.start_pos_endeffector[3], req.start_pos_endeffector[4], req.start_pos_endeffector[5]; //Punkt 1 von Niklas
      o2 << req.end_pos_endeffector[0], req.end_pos_endeffector[1], req.end_pos_endeffector[2], req.end_pos_endeffector[3], req.end_pos_endeffector[4], req.end_pos_endeffector[5];
      //o2 << 0.617488, 0.2473712, 0.251279, 0.0, 3.14, 0.0;
      //o2 << 0.417488, 0.2473712, 0.251279, 0.0,     3.14, 0.0;
      //o2 << 0.417488, 0.0473712, 0.251279, 0.0, 3.14, 0.0;
      //o2 << 0.436342, 0.109329, 0.103379, 0.0, 3.14, 0.0; // Punkt 2 von Niklas

      MatrixXd points(3,scaling_);
      points = getLinePoints(o1, o2);

      for (int i = 0; i<points.rows();i++){
        for(int j = 0; j<points.cols();j++){
          ROS_INFO(" Endeffektor Position %f", points(i,j));
        }
        ROS_INFO("----------------------");
      }


      //give inverse kinematics the current joint angles of the robot
      srv.request.initial_joint_angles = {init_position[0],
                                          init_position[1],
                                          init_position[2],
                                          init_position[3],
                                          init_position[4],
                                          init_position[5],
                                          init_position[6]};
      received_joint_angles[0] = {init_position[0],
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

        received_joint_angles[i+1] = {srv.response.ik_jointAngles[0],
                                      srv.response.ik_jointAngles[1],
                                      srv.response.ik_jointAngles[2],
                                      srv.response.ik_jointAngles[3],
                                      srv.response.ik_jointAngles[4],
                                      srv.response.ik_jointAngles[5],
                                      srv.response.ik_jointAngles[6]};
      }


      list<VectorXd> waypoints;
      VectorXd waypoint(6);
      VectorXd max_acc(6);
      VectorXd max_vel(6);
      for(int i = 0; i < received_joint_angles.size(); i++){
        for(int j = 0; j < received_joint_angles[i].size()-1; j++){
          waypoint[j] = received_joint_angles[i][j];
        }
        waypoints.push_back(waypoint);
      }
      for(int i = 0; i < 6; i++){
        max_acc[i] = max_accs[i];
        max_vel[i] = max_vels[i];
      }
      Trajectory trajectory(Path(waypoints, 0.1),max_vel, max_acc);
      trajectory.outputPhasePlaneTrajectory();
      array<vector<float>, 6> joints_smooth;
      trajectory_msgs::JointTrajectory traj_msg;
      trajectory_msgs::JointTrajectoryPoint pt;
      if(trajectory.isValid()) {
        double duration = trajectory.getDuration();
        ROS_INFO("Trajectory durrration: %f s", duration);
        for(double t = 0.0; t < duration; t += 0.001) {
          pt.positions.clear();
          for(int k = 0; k<joints_smooth.size(); k++) {
            joints_smooth[k].push_back(trajectory.getPosition(t)[k]);
            pt.positions.push_back(trajectory.getPosition(t)[k]);
          }
          pt.positions.push_back(init_position[6]);
          traj_msg.points.push_back(pt);
        }
      }
      ROS_INFO("Points : %i ", traj_msg.points.size());
      //std::reverse(traj_msg.points.begin(),traj_msg.points.end());
      traj_pub->publish(traj_msg);
      ros::spinOnce();
      /*for (int i=0; i<scaling_;i++){
          for (int j =0;j<7; j++){
              ROS_INFO("Joint Angles %f", received_joint_angles[i][j]);
          }
          ROS_INFO("--------------------------");
      }

      finalJointAngles = received_joint_angles[0];*/
    }
    };


int main(int argc, char** argv)
{
    // Init the ROS node
    ros::init(argc, argv, "needle_insertion");
    ros::NodeHandle nh("~");


    // create RobotArm object
    RobotArm arm(nh);


    // loop infinitely with a fixed frequency and send our commands
    ros::Rate loop_rate(1000);
    ros::ServiceServer service = nh.advertiseService("needle_insertion_service",&RobotArm::needle_insertion,&arm);
 // ros::ServiceServer service = node_handle.advertiseService("unserService",&IncrementalInverseKinematics::ik_jointAngles,&inverse_kinematics);
    while (ros::ok())
    {
        //arm.sendStepCommand();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
