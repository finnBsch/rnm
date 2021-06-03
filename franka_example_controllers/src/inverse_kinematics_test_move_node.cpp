//
// Created by tolga on 02.06.21.
//
/*
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <stdlib.h>
#include <math.h>
#include <sensor_msgs/JointState.h>
#include "inverse_kinematics/unserService.h"

class RobotArm
{

private:
    ros::NodeHandle nh_;
    std::vector<std::string> joint_names_;
    unsigned int num_joints_;
    std::string command_topic_;
    long counter = 0;
    std::array<double, 7> init_position{};
    ros::Publisher command_pub;
    std::vector<double> finalJointAngles;

public:

    // Initialize
    RobotArm(ros::NodeHandle nh, std::string command_topic): nh_(nh), num_joints_(7), command_topic_(command_topic_)
    {
        ros::ServiceClient client = nh.serviceClient<inverse_kinematics::unserService>("inverse_kinematics_node/unserService");
        inverse_kinematics::unserService srv;

        auto a = client.call(srv);
        if (a)
        {
            ROS_INFO("A-Matrix: %f", srv.response.ik_jointAngles);
        }
        else
        {
            ROS_ERROR("Failed to call service forward_kin");
            exit; //TODO find better solution
        }

        sensor_msgs::JointState joint_state_msg;
        finalJointAngles = {srv.response.ik_jointAngles[0],srv.response.ik_jointAngles[1],srv.response.ik_jointAngles[2],
                srv.response.ik_jointAngles[3],srv.response.ik_jointAngles[4],srv.response.ik_jointAngles[5],
                srv.response.ik_jointAngles[6]};

        if (command_topic.find("sim") != std::string::npos) {
            joint_state_msg  = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",ros::Duration(10)));
        }
        else {
            joint_state_msg  = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/franka_state_controller/joint_states_desired",ros::Duration(10)));
        }
        for (size_t i = 0; i < 7; ++i) {
            init_position[i] = joint_state_msg.position[i];
        }
        command_pub = nh_.advertise<std_msgs::Float64MultiArray>(command_topic, 1);
    }

    void sendStepCommand()
    {
        // calculate new joint angles
        // here it is just a sine wave on the initial joint angles
        std::vector<double> goal_position;
        std::vector<double> delta_angle= finalJointAngles;
        double incrementalCounter =counter/1000.;
            transform(delta_angle.begin(), delta_angle.end(), delta_angle.begin(),
                      [incrementalCounter](double &c) { return c * incrementalCounter; });
            //delta_angle =  finalJointAngles*counter/1000.;
            for (size_t i = 0; i < 7; ++i) {
                if (i == 4) {
                    goal_position.push_back(init_position[i] - delta_angle[i]);
                } else {
                    goal_position.push_back(init_position[i] + delta_angle[i]);
                }
            }
        if(incrementalCounter>1) {
            ros::shutdown();
        }

            counter++;


        // create message and publish it
        std_msgs::Float64MultiArray msg;
        msg.data.clear();
        msg.data.insert(msg.data.end(), goal_position.begin(), goal_position.end());
        command_pub.publish(msg);



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


    // loop infinitely with a fixed frequency and send our commands
    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        arm.sendStepCommand();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
*/