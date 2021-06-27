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


using namespace std;
using namespace cv;
using geometry_msgs::TransformStamped;

//// ws path
string path = "/home/nico/CLionProjects/rnm_ws";

//// calibration data path, contains: joints.txt, pose.txt, trk.txt
string pathCal = "/home/nico/Documents/RNM/Rosbag/calibration/calibration";

// container for camera calibration data
/*
std::vector<cv::Mat> cameraPosesR;
std::vector<cv::Mat> cameraPosesR_Mat;
std::vector<cv::Mat> cameraPosesT;
*/
cv::Mat joint_states;
vector<cv::Mat> R_gripper2base, t_gripper2base; // poses

// tf listener to get robot pose
tf::TransformListener* tfListener;
//tf2_ros::TransformListener tfListener;
cv::String robot_base = "panda_link0";
cv::String gripper = "panda_link8";

// read in text files into joint_states matrix
void readJointStates() {
  //// joint states
  try {
    ifstream file(pathCal + "/joints.txt");
    string line;
    while (getline(file, line)) {
      istringstream stream(line);
      double d;
      while (stream >> d) {
        joint_states.push_back(d);
      }
    }
    // rows++;
    cout << joint_states.size() << endl;
    joint_states = joint_states.reshape(1, 40); // 7 per row, 40 rows
    cout << "joint_states: " << joint_states << endl;
    ROS_INFO("Done reading joint states.");
  }
  catch (...) {
    ROS_INFO("Error while reading joint states!!!");
  }
}

void readEndPoses() {
  //// end-effector poses
  // reading: 40 lines, each line consists of 16 entries (4x4 homogeneous rot + trans matrix)
  // need to split into R matrix and t vector

  // first pushing R entries of one matrix to oneR_gripper2base, t equivalently
  // then pushing those matrices to the vectors of matrices (R_gripper2base, t_gri...)
  Mat oneR_gripper2base;
  Mat onet_gripper2base;
  try {
    int rot_count = 1; // 3 entries to R mat, next entry to t mat, 3 times, then toss next four entries (0 0 0 1), then repeat for next line
    ifstream file(pathCal + "/pose.txt");
    string line;
    while (getline(file, line)) {
      istringstream stream(line);
      double d;


      if (rot_count != 4 && rot_count != 8 && rot_count < 12) {
        while (stream >> d) {
          oneR_gripper2base.push_back(d);
          rot_count++;
        }
      }
      else if (rot_count < 12) {
        while (stream >> d) {
          onet_gripper2base.push_back(d);
          rot_count++;
        }
      }
      rot_count = 1;
      oneR_gripper2base = oneR_gripper2base.reshape(1, 3);
      onet_gripper2base = onet_gripper2base.reshape(1, 3);
      R_gripper2base.push_back(oneR_gripper2base);
      t_gripper2base.push_back(onet_gripper2base);
      cout << "oneR_gripper2base: " << oneR_gripper2base << endl;
      cout << "onet_gripper2base: " << onet_gripper2base << endl;
    }
    ROS_INFO("Done reading gripper to base transformation.");
  }
  catch (...) {
    ROS_INFO("Error while reading gripper to base transformation!!!");
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
  //readCheckPoses();

  //////////
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  tfListener = new tf::TransformListener();
  ///////////

  /* TODO: service that publishes matrix<xd> of defined size containing hand eye matrix*/

  return 0;
}