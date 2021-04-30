#include <ros/ros.h>
#include "sensor_msgs/JointState.h"

void callback(const sensor_msgs::JointState& msg) {
  std::cout << "Joints: ";
  for (int i = 0; i < 7; ++i ) {
    std::cout << msg.position[i]*180/M_PI << " ";
  }
  std::cout << "\n\n";
}

int main(int argc, char** argv)  {
  ros::init(argc, argv, "panda_joint_reader_node");

  // Get a node handle to the private parameters
  // e.g. /node_name/parameter_name can then be
  // read by nh.getParam("parameter_name", value);
  ros::NodeHandle  nh("~");

  // Parse parameters specified in the .launch file
  std::string topic_name;
  int queue_size;
  nh.getParam("topic_name", topic_name);
  nh.getParam("queue_size", queue_size);

  // Register a callback function (a function that is called every time a new message arrives)
  ros::Subscriber sub = nh.subscribe("/joint_states", 10, callback);

  // Prevent the program from terminating
  // Blocks but still allows messages to be received
  ros::spin();
}