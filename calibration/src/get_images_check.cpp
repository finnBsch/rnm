#include <image_transport/image_transport.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

using namespace std;

// create publishers globally
ros::Publisher rgb_pub;
ros::Publisher ir_pub;
ros::Publisher joint_states_pub;

// is being set true if joinState message is correct, set false again right after publishing
bool success = false;

//// callbacks:
// fetch current image from sensor_msgs and publish it
void fetchRgbImg(const sensor_msgs::ImageConstPtr& rgbmsg) {
  while (success != 1) {
    if (success == 1) {
      try {
        rgb_pub.publish(rgbmsg);
        cout << "rgb message published successfully" << endl;
      } catch (const std::exception&) {
        ROS_ERROR("Could not publish rgb message !!!!");
      }
    }
  }
}

void fetchIrImg(const sensor_msgs::ImageConstPtr& irmsg) {
  try {
    ir_pub.publish(irmsg);
    cout << "ir  message published successfully" << endl;
  } catch (const std::exception&) {
    ROS_ERROR("Could not publish ir message !!!!");
  }
}

void fetchJointStates(const sensor_msgs::JointState& msg) {
  // 3/4 of all joint_states/position messages are corrupted
  // need to check if message is good before publishing it
  vector<double> one_joint_states;
  one_joint_states = msg.position;
  auto threshold = float(2);
  auto sum = float(0);


  while (success != 1) {
    for (int i = 0; i < one_joint_states.size(); i++) {
      sum += fabs(float(one_joint_states[i]));
    }
    cout << "sum: " << sum << endl;
    if (sum > threshold) {
      try {
        joint_states_pub.publish(msg);
        cout << "joint_states  message published successfully" << endl;
        success = 1;
      } catch (const std::exception&) {
        ROS_ERROR("Could not publish joint_state message !!!!");
      }
    }
  }
  success = 0;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "get_images");
  ros::NodeHandle n; // node initialisation
  image_transport::ImageTransport it(n);

  // subscribe to camera topics
  // callbacks to fetch current images
  //image_transport::Subscriber rgb_sub = it.subscribe("/rgb/image_raw", 1, fetchRgbImg); // reality
  image_transport::Subscriber rgb_sub = it.subscribe("k4a/rgb/image_raw", 1, fetchRgbImg); //rosbag
  //image_transport::Subscriber ir_sub = it.subscribe("/ir/image_raw", 1, fetchIrImg);
  //image_transport::Subscriber ir_sub = it.subscribe("k4a/ir/image_raw", 1, fetchIrImg);
  ros::Subscriber joint_states_sub = n.subscribe("/joint_states", 1, fetchJointStates);
  //ros::Subscriber joint_states_sub = n.subscribe("/franka_state_controller/joint_states_desired", 1, fetchJointStates);

  // initialise publishers
  rgb_pub = n.advertise<sensor_msgs::Image>("calibration_rgb_img", 1);
  ir_pub = n.advertise<sensor_msgs::Image>("calibration_ir_img", 1);
  joint_states_pub = n.advertise<sensor_msgs::JointState>("calibration_joint_states", 1);

  while (ros::ok()) {
    cout << "Press 'ENTER' to select most recent frame" << endl;
    // wait until any key gets pressed
    while (cin.ignore()) {
      ros::spinOnce();
    }
  }
  return 0;
}