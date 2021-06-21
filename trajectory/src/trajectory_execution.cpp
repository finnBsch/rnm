//
// Created by finn on 07.06.21.
//

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "std_msgs/Float64MultiArray.h"
class trajectory_executer{
public:
    ros::Publisher pub;
    trajectory_executer(ros::Publisher traj):pub(traj){}
    void execute_trajectory(const trajectory_msgs::JointTrajectoryConstPtr& msg){
        ros::Rate loop_rate(1000);
        std_msgs::Float64MultiArray msg_;
        bool not_finished = true;
        int i = 0;
        while(ros::ok() && not_finished){
            if(msg->points[i].positions.size() ==7) {
                msg_.data.clear();
                msg_.data.insert(msg_.data.end(), msg->points[i].positions.begin(), msg->points[i].positions.end());
                pub.publish(msg_);
            }
            i++;
            if(i==msg->points.size()){
                not_finished = false;
            }
            loop_rate.sleep();
        }

    }

};


int main(int argc, char **argv){
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    // TODO differ between sim
    ros::Publisher trajectory_publisher_sim = n.advertise<std_msgs::Float64MultiArray>("/joint_position_example_controller_sim/joint_command", 1000);
    ros::Publisher trajectory_publisher = n.advertise<std_msgs::Float64MultiArray>("/joint_position_example_controller/joint_command", 1000);
    trajectory_executer traj_exec(trajectory_publisher);
    trajectory_executer traj_exec_sim(trajectory_publisher_sim);
    ros::Subscriber sub = n.subscribe("trajectory", 5, &trajectory_executer::execute_trajectory,&traj_exec);
    ros::Subscriber sub_sim = n.subscribe("trajectory_sim", 5, &trajectory_executer::execute_trajectory,&traj_exec_sim);
    ros::spin();
}