//
// Created by nico on 18.06.21.
//
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
#include "OCVcalib3d.hpp"  // header file that includes the calibrateHandEye method
#include "OCVcalibration_handeye.cpp"
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>

//using namespace cv;
using namespace std;

// define number of frames  to be extracted and used for calibration... later on this will be number of good frame pairs
int n_frames = 10;

// package path
string path = "/home/nico/catkin_ws/src";

// counters for the filenames
int pose_rgb = 1;
int pose_ir = 1;
int js_count = 0;

// container for camera calibration data
std::vector<cv::Mat> rvecs, tvecs;

// container for poses as geometry_msgs
vector<geometry_msgs::TransformStamped> allRobotPoses;

// tf listener to get robot pose
tf2_ros::Buffer tfBuffer;
//tf2_ros::TransformListener tfListener;
cv::String robot_base, gripper;

// saving jointState msg
void jointStatesWrite(const sensor_msgs::JointState& msg) {
  // get robot pose
  geometry_msgs::TransformStamped robot_pose = tfBuffer.lookupTransform(robot_base, gripper, ros::Time(0));
  allRobotPoses.push_back(robot_pose);
  js_count++;
}

// ir image writing
void irImageWrite(const sensor_msgs::ImageConstPtr& msg) {
  try {
    // convert image from ros msg to bgr8 (cv)
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;


    // create filename
    stringstream ss;
    string name = "/frame_reader/cal_imgs/new/ir/pose_";
    string type = ".jpg";
    ss << path << name << (pose_ir) << type; // pose = counter, set at start
    string filename = ss.str();
    ss.str("");
    // write img
    imwrite(filename, img);
    std::cout << "ir: Pose_" << pose_ir << " img written." << endl;
    pose_ir++;
  }
  catch (cv_bridge::Exception &e) {
    std::cout << "Could not write ir: Pose_" << pose_ir-1 << " image." << endl;
  }
}

// rgb image writing
void rgbImageWrite(const sensor_msgs::ImageConstPtr& msg) {
  try {
    // convert image from ros msg to bgr8 (cv)
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

    // create filename
    stringstream ss;
    string name = "/frame_reader/cal_imgs/new/rgb/pose_";
    string type = ".jpg";
    ss << path << name << (pose_rgb) << type; // pose = counter, set at start
    string filename = ss.str();
    ss.str("");
    // write img
    imwrite(filename, img);
    std::cout << "rgb: Pose_" << pose_rgb << " img written." << endl;
    pose_rgb++;
  }
  catch (cv_bridge::Exception &e) {
    std::cout << "Could not write rgb: Pose_" << pose_rgb-1 << " image." << endl;
  }
}

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

// two transformation functions that are going to be used in handEye()
void quat2angaxis(double x, double y, double z, double w, cv::Mat& angaxis) {
  // normorlize quaternion
  double norm = sqrt(x*x + y*y + z*z + w*w);
  x = x/norm;
  y = y/norm;
  z = z/norm;
  w = w/norm;
  // convert to angle axis
  double angle = 2*acos(w);
  double s = sqrt(1-w*w);
  double rx, ry, rz;
  if(s < 0.00001)
  {
    // angle is small, so rotation direction is not important
    rx = 1;
    ry = 0;
    rz = 0;
  }
  else
  {
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

void angaxis2quat(cv::Mat angaxis, vector<double>& quat)
{
  double rx = angaxis.at<double>(0,0);
  double ry = angaxis.at<double>(1,0);
  double rz = angaxis.at<double>(2,0);
  double angle = sqrt(rx*rx + ry*ry + rz*rz);
  quat = vector<double>{0,0,0,0};
  quat[0] = rx/angle*sin(angle/2);
  quat[1] = ry/angle*sin(angle/2);
  quat[2] = rz/angle*sin(angle/2);
  quat[3] = cos(angle/2);
}

void transform2rv(geometry_msgs::TransformStamped transform, cv::Mat& rvec, cv::Mat& tvec){
  tvec = cv::Mat::zeros(3,1,CV_64F);
  rvec = cv::Mat::zeros(3,1,CV_64F);
  auto rot = transform.transform.rotation;
  auto tran = transform.transform.translation;
  quat2angaxis(rot.x, rot.y, rot.z, rot.w, rvec);
  tvec.at<double>(0,0) = tran.x;
  tvec.at<double>(1,0) = tran.y;
  tvec.at<double>(2,0) = tran.z;
  return;
}

// perform hand-eye calibration with the calculated transforms
void handEye(){

  vector<cv::Mat> R_gripper2base, t_gripper2base;
  for(int i=0; i<allRobotPoses.size()-1; i++)
  {
    cv::Mat R, t;
    transform2rv(allRobotPoses[i], R, t);
    R_gripper2base.push_back(R);
    t_gripper2base.push_back(t);
  }
  cv::Mat R_cam2gripper, t_cam2gripper;

  ROS_INFO("Starting hand-eye calibration.");

  calibrateHandEye(R_gripper2base, t_gripper2base, rvecs, tvecs, R_cam2gripper, t_cam2gripper);
  cout << "hand eye transformation: translation:\n" << t_cam2gripper << endl;
  cout << "hand eye transformation: rotation:\n" << R_cam2gripper << endl;

  // convert rotation matrix to angle axis
  Rodrigues(R_cam2gripper, R_cam2gripper);
  // convert angle axsi to quaternion
  vector<double> q;
  angaxis2quat(R_cam2gripper, q);

  // saving calibration results
  ofstream myfile;
  myfile.open("camera_hand_eye_calibration.yaml");
  myfile << "# This is autogenerated file to store camera and hand-eye calibration results\n";
  myfile << "camera_calibration:\n";
  myfile << "  # camera matrix\n";
  //myfile << "  K: " << camera_matrix.reshape(1,1) << endl;
  myfile << "  # distortion parameters\n";
  //myfile << "  D: " << dist_coeffs.reshape(1,1) << endl;
  myfile << "hand_eye_position:\n";
  myfile << "  # rotaion matrix\n";
  myfile << "  rotation: [" << q[0] << ", " << q[1] << ", ";
  myfile << q[2] << ", " << q[3] << "]" << endl;
  myfile << "  # translation\n";
  myfile << "  translation: " << t_cam2gripper.reshape(1,1) << endl;
  ROS_INFO("calibration saved to 'camera_hand_eye_calibration.yaml");
}

int main(int argc, char** argv) {
  ros::init(argc, argv,"calibration_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  // callbacks. each image type has its own saving callback
  image_transport::Subscriber rgb_sub = it.subscribe("/calibration_rgb_img", 1, rgbImageWrite);
  image_transport::Subscriber ir_sub = it.subscribe("/calibration_ir_img", 1, irImageWrite);
  ros::Subscriber joint_states_sub = nh.subscribe("/calibration_joint_states", 1, jointStatesWrite);

  std::cout << "\nready to receive frames" << endl;
  while(js_count < 10+1) {
    ros::spinOnce();
  }
  cout << "\n collected " << n_frames << " frames" << endl;
  cout << "starting camera calibration" << endl;
  cameraCalibration();
  handEye();


  return 0;
}
