#include "ros/ros.h"
#include "forward_kin/get_endeffector.h"
#include <sensor_msgs/JointState.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "forward_kin_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<forward_kin::get_endeffector>("forward_kin_node/get_endeffector");
  forward_kin::get_endeffector srv;
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg  = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",ros::Duration(10)));
  boost::array<double, 7> arr = {joint_state_msg.position[0], joint_state_msg.position[1],joint_state_msg.position[2],
                                 joint_state_msg.position[3], joint_state_msg.position[4],joint_state_msg.position[5],
                                 joint_state_msg.position[6]};
  srv.request.joint_angles = arr;
  auto a = client.call(srv);
  if (a)
  {
    ROS_INFO("Endpos: %f", srv.response.end_effector_pos[0]);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}