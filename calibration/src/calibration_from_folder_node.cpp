/*
 * This file can be used to verify calibration and transformation algorithms.
 * It reads the provided text files and feeds them into the calibration.
*/
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <opencv2/imgcodecs.hpp>
#include "cv_handeye.h"
#include <tf/transform_listener.h>
#include <opencv2/core/mat.hpp>


using namespace std;
using namespace cv;
using geometry_msgs::TransformStamped;

//// ws path
string path = "/home/nico/CLionProjects/rnm_ws";

//// calibration data path, contains: joints.txt, pose.txt, trk.txt
//string pathCal = "/home/nico/Documents/RNM/Rosbag/calibration/calibration";
string pathCal = "/home/nico/cal_data/calibration";

cv::Mat joint_states;
vector<cv::Mat> R_gripper2base, t_gripper2base; // all gripper poses
vector<cv::Mat> R_target2cam, t_target2cam; // all checkerboard poses

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
  try {
    int rot_count = 1; // for each row, first 3 entries to R mat, 4th entry to t mat, repeat 3 times, then toss next four entries (0 0 0 1), 16 in total, then repeat for next line
    ifstream file(pathCal + "/pose.txt");
    string line;
    while (getline(file, line)) {
      istringstream stream(line);
      double d;
      Mat oneR_gripper2base;
      Mat onet_gripper2base;
      while (stream >> d) {
        if (rot_count == 4 || rot_count == 8 || rot_count == 12) {
          onet_gripper2base.push_back(d);
          rot_count++;
        }
        else if (rot_count < 12){
          oneR_gripper2base.push_back(d);
          rot_count++;
        }
        if (rot_count == 13) {
          break;
        }
      }
      oneR_gripper2base = oneR_gripper2base.reshape(1, 3);
      //cout << "oneR_gripper2base: " << oneR_gripper2base << endl;
      //cout << "onet_gripper2base: " << onet_gripper2base << endl;
      t_gripper2base.push_back(onet_gripper2base);
      R_gripper2base.push_back(oneR_gripper2base);
      rot_count = 1;
    }
    cout << "R_gripper2base.size: " << R_gripper2base.size() << endl;
    cout << "t_gripper2base.size: " << t_gripper2base.size() << endl;
    ROS_INFO("Done reading gripper to base transformation.");
  }
  catch (...) {
    ROS_INFO("Error while reading gripper to base transformation!!!");
  }
}

void readCheckPoses() {
  //// checkerboard poses
  // reading: 40 lines, each line consists of 16 entries (4x4 homogeneous rot + trans matrix)
  // need to split into R matrix and t vector

  // first pushing R entries of one matrix to oneR_target2cam, t equivalently
  // then pushing those matrices to the vectors of matrices (R_target2cam, t_tar...)
  try {
    int rot_count = 1; // for each row, first 3 entries to R mat, 4th entry to t mat, repeat 3 times, then toss next four entries (0 0 0 1), 16 in total, then repeat for next line
    ifstream file(pathCal + "/trk.txt");
    string line;
    while (getline(file, line)) {
      istringstream stream(line);
      double d;
      Mat oneR_target2cam;
      Mat onet_target2cam;
      while (stream >> d) {
        if (rot_count == 4 || rot_count == 8 || rot_count == 12) {
          onet_target2cam.push_back(d);
          rot_count++;
        }
        else if (rot_count < 12){
          oneR_target2cam.push_back(d);
          rot_count++;
        }
        if (rot_count == 13) {
          break;
        }
      }
      oneR_target2cam = oneR_target2cam.reshape(1, 3);
      //cout << "oneR_target2cam: " << oneR_target2cam << endl;
      //cout << "onet_target2cam: " << onet_target2cam << endl;
      t_target2cam.push_back(onet_target2cam);
      R_target2cam.push_back(oneR_target2cam);
      rot_count = 1;
    }
    cout << "R_target2cam.size: " << R_target2cam.size() << endl;
    cout << "t_target2cam.size: " << t_target2cam.size() << endl;
    ROS_INFO("Done reading checkerboard to camera transformation.");
  }
  catch (...) {
    ROS_INFO("Error while reading checkerboard to camera transformation!!!");
  }
}

void calculateTransforms(){
  vector<cv::Mat> R_gripper2base, t_gripper2base;
}

void HandEye() {
  //// hand eye
  ROS_INFO("Starting hand-eye-calibration...");
  Mat R_cam2gripper, t_cam2gripper;
  calibrateHandEye_(R_gripper2base,t_gripper2base, R_target2cam, t_target2cam,R_cam2gripper, t_cam2gripper);
  cout << "HandEye result \n" << "R_cam2gripper:\n" << R_cam2gripper << endl;
  cout << "t_cam2gripper:\n" << t_cam2gripper << endl;

  // saving results
  Mat r = R_cam2gripper;
  ofstream myfile;
  myfile.open(pathCal + "/camera_hand_eye_calibration.yaml");
  myfile << "# This is an autogenerated file to store camera and hand-eye calibration results\n";
  myfile << "camera_calibration:\n";
  myfile << "hand_eye_pose:\n";
  myfile << "  # rotation matrix\n";
  myfile << "  rotation: " << R_cam2gripper << endl;
  myfile << "  # translation\n";
  myfile << "  translation: " << t_cam2gripper.reshape(1,1) << endl;
  cout << "\n\tresults saved to '" << pathCal << "/camera_hand_eye_calibration.yaml'" << endl << endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "calibration_from_folder_node");

  //readJointStates();
  readEndPoses();
  readCheckPoses();
  HandEye();

  /*
  //////////
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  tfListener = new tf::TransformListener();
  ///////////
   */

  /* TODO: service that publishes matrix<xd> of defined size containing hand eye matrix*/

  return 0;
}