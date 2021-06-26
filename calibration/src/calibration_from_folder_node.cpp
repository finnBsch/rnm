#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include "cv_handeye.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <opencv2/core/mat.hpp>
#include <boost/filesystem.hpp>

using namespace std;
using namespace cv;
using geometry_msgs::TransformStamped;
namespace fs = boost::filesystem;

//// ws path
//fs::path path = "/home/nico/CLionProjects/rnm_ws";
string path = "/home/nico/CLionProjects/rnm_ws";

//// calibration data path, contains: joints.txt, pose.txt, trk.txt
//fs::path pathCal = "/home/nico/Documents/RNM/Rosbag/calibration/calibration";
string pathCal = "/home/nico/Documents/RNM/Rosbag/calibration/calibration";

// container for camera calibration data
std::vector<cv::Mat> cameraPosesR;
std::vector<cv::Mat> cameraPosesR_Mat;
std::vector<cv::Mat> cameraPosesT;
cv::Mat joint_states;
vector<cv::Mat> R_gripper2base, t_gripper2base;

// container for joint states
vector<tf::StampedTransform> allJointStates;

// tf listener to get robot pose
tf::TransformListener* tfListener;
//tf2_ros::TransformListener tfListener;
cv::String robot_base = "panda_link0";
cv::String gripper = "panda_link8";

// read in text files into joint_states matrix
void readJointStates() {
  //// joint states
  try {
    // ifstream file("/home/nico/Documents/RNM/Rosbag/calibration/calibration/joints.txt");
    ifstream file(pathCal + "joints.txt");
    // string file_line;
    int rows = 0;
    string line;
    for (line; getline(file, line);) {
      std::istringstream stream(line);
      // char separator = " ";


      // under maintenance
      string separator = " "; // not sure if this works
      double x;
      // read *both* a number and space:
      while (stream >> x && stream >> separator) {
        joint_states.push_back(x);
      }
      rows++;
    }
    joint_states = joint_states.reshape(1, rows);
    for (int i = 0; i<joint_states.rows; i++)
      //double row = joint_states.row(i);
      //allJointStates.push_back(joint_states.row(i));


    ROS_INFO("Done reading joint states.");
  }
  catch (...) {
    ROS_INFO("Error while reading joint states!!!");
  }
}

void readEndPoses() {
  //// end-effector poses
  // gripper to base
  try {
    ifstream file(pathCal + "poses.txt");
    // string file_line;
    int rows = 0;
    string line; // 16 entries
    for (line; getline(file, line);) {
      std::istringstream stream(line);
      // char separator = " ";
      string separator = " ";
      double x;
      // read *both* a number and space:
      while (stream >> x && stream >> separator) {
        joint_states.push_back(x);
      }
      rows++;
    }
    joint_states = joint_states.reshape(1, rows);
    ROS_INFO("Done reading end-effector poses.");
  }
  catch (...) {
    ROS_INFO("Error while reading end-effector poses!!!");
  }
}

void readCheckPoses() {
  //// checkerboard poses
  try {
    ifstream file(pathCal + "trk.txt");
    // string file_line;
    int rows = 0;
    string line;
    for (line; getline(file, line);) {
      std::istringstream stream(line);
      // char separator = " ";
      string separator = " ";
      double x;
      // read *both* a number and space:
      while (stream >> x && stream >> separator) {
        joint_states.push_back(x);
      }
      rows++;
    }
    joint_states = joint_states.reshape(1, rows);
    ROS_INFO("Done reading checkerboard poses.");
  }
  catch (...) {
    ROS_INFO("Error while reading checkerboard poses!!!");
  }
}

void calculateTransforms(){
  vector<cv::Mat> R_gripper2base, t_gripper2base;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "calibration_from_folder_node");

  readJointStates();
  readEndPoses();
  readCheckPoses();

  //////////
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  tfListener = new tf::TransformListener();
  ///////////

  /* TODO: service that publishes matrix<xd> of defined size containing hand eye matrix*/

  return 0;
}