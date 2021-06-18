//
// Created by nico on 11.06.21.
//
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <iostream>

#include <fstream>

#include <opencv2/imgcodecs.hpp>
#include "OCVcalib3d.hpp"  // header file that includes the calibrateHandEye method
#include "OCVcalibration_handeye.cpp"
#include "cv_bridge/cv_bridge.h"
#include <cstdlib>
//#include <opencv2/calib3d.hpp>
//#include <opencv2/imgproc.hpp>
#include <sstream>
#include <string>
#include "forward_kin/get_endeffector.h"
#include "geometry_msgs/Pose.h"
#include "image_transport/image_transport.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

using namespace std;
//using namespace cv;
using geometry_msgs::TransformStamped;

/*
 * first part: pseudo calibration to generate plausible rvecs + tvecs
 * second part: hand-eye calibration
 */

// container for camera calibration data
std::vector<cv::Mat> rvecs, tvecs;

// pseudo rgb calibration to create rvecs + tvecs
int cameraCalibration() {
  //int argc, char **argv
  //(void)argc;
  //(void)argv;
  ROS_INFO("Starting pseudo camera calibration.");
  std::vector<cv::String> fileNames;
  cv::glob("/home/nico/catkin_ws/src/frame_reader/cal_imgs/new/rgb/pose_*.jpg", fileNames, false);
  cv::Size patternSize(9 - 1, 6 - 1);
  std::vector<std::vector<cv::Point2f>> q(fileNames.size());

  std::vector<std::vector<cv::Point3f>> Q;
  // 1. Generate checkerboard (world) coordinates Q. The board has 9x6 squares
  // fields with a size of 44mm
  int checkerBoard[2] = {9,6};
  int fieldSize = 44;
  // Defining the world coordinates for 3D points
  std::vector<cv::Point3f> objp;
  for(int i = 1; i<checkerBoard[1]; i++){
    for(int j = 1; j<checkerBoard[0]; j++){
      objp.push_back(cv::Point3f(j*fieldSize,i*fieldSize,0));
    }
  }
  std::vector<cv::Point2f> imgPoint;
  // Detect feature points on checkerboard
  std::size_t i = 0;
  for (auto const &f : fileNames) {
    std::cout << std::string(f) << std::endl;
    // 2. Read images
    cv::Mat img = cv::imread(fileNames[i]);
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);
    // if all points found: grayscale + pattern size + cv stuff --> camera coordinates q
    bool patternFound = cv::findChessboardCorners(gray, patternSize, q[i], cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
    // 2. Use cv::cornerSubPix() to refine the found corner detections
    // if found: push to Q
    if(patternFound){
      cv::cornerSubPix(gray, q[i],cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
      Q.push_back(objp);
    }
    i++;
  }
  // outputs from calibrateCamera
  cv::Matx33f K(cv::Matx33f::eye());  // intrinsic camera matrix
  cv::Vec<float, 5> k(0, 0, 0, 0, 0); // distortion coefficients
  //cv::Vec<float, 8> k(0, 0, 0, 0, 0,0,0,0); // distortion coefficients

  // rotation + translation vector
  //std::vector<cv::Mat> rvecs, tvecs;
  std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
  // opencv stuff
  int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 +
              cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;
  cv::Size frameSize(2048, 1536);//
  std::cout << "Calibrating..." << std::endl;
  float error = cv::calibrateCamera(Q, q, frameSize, K, k, rvecs, tvecs, flags);
  std::cout << "Reprojection error = " << error << "\nK =\n"
            << K << "\nk=\n"
            << k << std::endl;
  // Precompute lens correction interpolation
  cv::Mat mapX, mapY;
  cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1,
                              mapX, mapY);
  return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * hand-eye calibration
 */

// containers for calibration data
//vector<TransformStamped> allRobotPoses;
cv::Mat joint_states;
vector<cv::Mat> R_gripper2base, t_gripper2base;

vector<vector<cv::Point2f>> allCharucoCorners;
vector<vector<int>> allCharucoIds;

// read in text files into joint_states matrix
void readCalibrationData() {
  ifstream file("/home/nico/CLionProjects/rnm_ws/src/calibration/src/joint_states.txt");
  //string file_line;
  int rows = 0;
  //file.open("joint_states.txt");
  //while (getline(file, line)) {
  string line;
  for( line; getline(file, line); ) {
    std::istringstream stream(line);
    char sep;  // comma!
    double x;
    // read *both* a number and a comma:
    while (stream >> x && stream >> sep) {
      joint_states.push_back(x);
      //cout << joint_states <<endl;
    }
    rows++;
  }
  joint_states = joint_states.reshape(1, rows);
  //cout << joint_states <<endl;
  ROS_INFO("Done reading calibration data.");
}

// quaternion to angles conversion, used in transform2rv()
// credits to https://github.com/xiaohuits/camera_hand_eye_calibration
void quat2angaxis(double x, double y, double z, double w, cv::Mat& angaxis){
  // normarlise quaternion
  double norm = sqrt(x*x + y*y + z*z + w*w);
  x = x/norm;
  y = y/norm;
  z = z/norm;
  w = w/norm;
  // convert to angle axis
  double angle = 2*acos(w);
  double s = sqrt(1-w*w);
  double rx, ry, rz;
  if(s < 0.00001) {
    // angle is small, so rotation direction is not important
    rx = 1;
    ry = 0;
    rz = 0;
  }
  else {
    rx = x / s;
    ry = y / s;
    rz = z / s;
  }
  // normalize rotation direction
  norm = sqrt(rx*rx + ry*ry + rz*rz);
  rx = rx / norm * angle;
  ry = ry / norm * angle;
  rz = rz / norm * angle;
  angaxis.at<double>(0,0) = rx;
  angaxis.at<double>(1,0) = ry;
  angaxis.at<double>(2,0) = rz;
}


// only 3 indeces need 7 for this conversion


// calculating transformation from pose
// credits to https://github.com/xiaohuits/camera_hand_eye_calibration
//void transform2rv(TransformStamped transform, Mat& rvec, Mat& tvec){
void transform2rv(boost::array<double, 3> pose, cv::Mat& rvec, cv::Mat& tvec){ // 3 to 8
  tvec = cv::Mat::zeros(3,1,CV_64F);
  rvec = cv::Mat::zeros(3,1,CV_64F);
  //auto rot = transform.transform.rotation;
  //auto tran = transform.transform.translation;
  //quat2angaxis(rot.x, rot.y, rot.z, rot.w, rvec);
//////////////////////////////////////////////////////////////////// fix dis
  //quat2angaxis(  pose[3],pose[4], pose[5], pose[6], rvec); // was 4,5,6,7 initially
  tvec.at<double>(0,0) = pose[0];
  tvec.at<double>(1,0) = pose[1];
  tvec.at<double>(2,0) = pose[2];
  // tvec.at<double>(0,0) = tran.x;
  //   tvec.at<double>(1,0) = tran.y;
  //   tvec.at<double>(2,0) = tran.z;
  return;
}

// calculate poses from joint_states using forward_kin/get_endeffector
// then get the transformations to these poses
int calculatePose() {
  vector<boost::array<double, 3>> poses;//(joint_states.rows); // 3 to 7
  //(void) argc;
  //(void) argv;
  // ros::init(argc, argv, "hand_eye_calibration");
  // sensor_msgs::JointState joint_state_msg;
  // joint_state_msg  = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",ros::Duration(10)));

  // create client object
  ros::NodeHandle n;
  ros::ServiceClient client =
      n.serviceClient<forward_kin::get_endeffector>("forward_kin_node/get_endeffector");
  forward_kin::get_endeffector srv;

  // put joint angles into arr
  for (int i = 0; i < joint_states.rows; i++) {
    std::vector<double> row = joint_states.row(i);
    /*for(size_t i = 7; i--;) { // just for debugging
      cout << row[i] << endl;
    }*/
    boost::array<double, 7> arr = {row.at(0), row.at(1), row.at(2), row.at(3),
                                   row.at(4), row.at(5), row.at(6)};

    // request service for conversion
    srv.request.joint_angles = arr;
    auto a = client.call(srv);
    if (a) {
      //// need to push back the pose (transformation) to allRobotPoses
      boost::array<double, 3> pose = srv.response.end_effector_pos; //changed 3 to 8
      poses.push_back(pose);
      // allRobotPoses.transform.rotation.x[i];
      // allRobotPoses.transform.translation.push_back(pose);
    } else {
      ROS_ERROR("Failed to call service forward_kin");
      return 1;
    }
  }
    // now, use transform2rv to calculate transformations
  for (int i = 0; i < poses.size(); i++) {
    cv::Mat R, t;
    transform2rv(poses[i], R, t);
    R_gripper2base.push_back(R);
    t_gripper2base.push_back(t);
  }
  ROS_INFO("Done calculating transforms.");
}

// perform hand-eye calibration with the calculated transforms
void handEye(){
    ROS_INFO("Starting hand-eye calibration.");
    cv::Mat R_cam2gripper, t_cam2gripper;
    calibrateHandEye(R_gripper2base, t_gripper2base, rvecs, tvecs, R_cam2gripper, t_cam2gripper, cv::CALIB_HAND_EYE_TSAI);
    cout << "hand eye transformation: translation:\n" << t_cam2gripper << endl;
    cout << "hand eye transformation: rotation:\n" << R_cam2gripper << endl;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "hand_eye_calibration");
    cameraCalibration();
    readCalibrationData();
    calculatePose();
    handEye();
    /*
    Calibrator obj;
    ros::Rate loop_rate(50);
    cout << "press 's' to add current keyframe, 'c' to calibrate, 'q' to quit program" << endl;
    while (ros::ok()){
        int key = obj.showImage();
        if(key == 's')
        {
            obj.saveData();
        }
        if(key == 'c')
        {
            obj.calibrate();
        }
        if(key == 'q')
        {
            break;
        }
        ros::spinOnce();
    }
     */
    //ros::spin();
    return 0;
}