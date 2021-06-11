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
            msg_.data.clear();
            msg_.data.insert(msg_.data.end(), msg->points[i].positions.begin(), msg->points[i].positions.end());
            pub.publish(msg_);
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
    ros::Publisher trajectory_publisher = n.advertise<std_msgs::Float64MultiArray>("/joint_position_example_controller_sim/joint_command", 1000);
    trajectory_executer traj_exec(trajectory_publisher);
    ros::Subscriber sub = n.subscribe("trajectory", 5, &trajectory_executer::execute_trajectory,&traj_exec);
    ros::spin();
}