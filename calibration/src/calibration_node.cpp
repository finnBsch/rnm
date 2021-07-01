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
#include "cv_handeye.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>

#include <opencv2/core/mat.hpp>

#include <tf2_ros/transform_listener.h>

using namespace cv;
using namespace std;

// define number of frames  to be extracted and used for calibration... later on this will be number of good frame pairs
int n_frames = 10;

// package path
string path = "/home/nico/cal_data";
//string path = "/home/rnm_grp1/rgb";

// counters for the filenames
int pose_rgb = 1;
int pose_ir = 1;
int js_count = 0;
int rgb_count = 0;

// container for camera calibration data
//std::vector<cv::Mat> rvecs, tvecs;
std::vector<cv::Mat> cameraPosesR;
std::vector<cv::Mat> R_target2cam;
std::vector<cv::Mat> t_target2cam;
// container for joint states as geometry_msgs
vector<tf::StampedTransform> allJointStates;

// tf listener to get robot pose
tf::TransformListener* tfListener;
//tf2_ros::TransformListener tfListener;
cv::String robot_base = "panda_link0";
cv::String gripper = "panda_link8";

// saving jointState msg
void jointStatesWrite(const sensor_msgs::JointState& msg) {
  if(js_count<n_frames) {
    // get robot pose
    tf::StampedTransform joint_state;
    tfListener->lookupTransform(robot_base, gripper, ros::Time(0), joint_state);
    allJointStates.push_back(joint_state);
    js_count++;
  }
}

// ir image writing
void irImageWrite(const sensor_msgs::ImageConstPtr& msg) {
  try {
    // convert image from ros msg to bgr8 (cv)
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;


    // create filename
    stringstream ss;
    string name = "/pose_";
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
    std::cout << "Could not write ir: pose_" << pose_ir-1 << " image." << endl;
  }
}

// rgb image writing
void rgbImageWrite(const sensor_msgs::ImageConstPtr& msg) {
  try {
    if(rgb_count < n_frames) {
      // convert image from ros msg to bgr8 (cv)
      cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

      // create filename
      stringstream ss;
      string name = "/pose_";
      string type = ".jpg";
      ss << path << name << (pose_rgb) << type;  // pose = counter, set at start
      string filename = ss.str();
      ss.str("");
      // write img
      imwrite(filename, img);
      std::cout << "rgb: pose_" << pose_rgb << " img written." << endl;
      pose_rgb++;
      rgb_count++;
    }
  }
  catch (cv_bridge::Exception &e) {
    std::cout << "Could not write rgb: pose_" << pose_rgb-1 << " image." << endl;
  }
}

// camera calibration
int cameraCalibration() {
  /* TODO: make sure joint states are synced with images*/
  cv::Size rgbFrameSize(2048, 1536);
  cv::Size irFrameSize(640, 576);

  std::vector<cv::String> rgbFileNames(n_frames);
  for(int i = 0; i<n_frames; i++){
    rgbFileNames[i] = "/home/nico/cal_data/pose_" + to_string(i+1) + ".jpg";
  }
  //std::vector<cv::String> irFileNames;
  // std::string rgbFolder("/home/lars/CLionProjects/CameraCalibrationtests/cal_imgs/rgb/*.jpg");
  //std::string rgbFolder("/home/nico/catkin_ws/src/frame_reader/cal_imgs/new/rgb/*.jpg");
  //cv::glob(rgbFolder, rgbFileNames, true);  // load rgb images into opencv
  //std::string irFolder("/home/nico/catkin_ws/src/frame_reader/cal_imgs/new/ir/*.jpg");
  // std::string irFolder("/home/lars/CLionProjects/CameraCalibrationtests/cal_imgs/ir/*.jpg");
  //cv::glob(irFolder, irFileNames, true);

  //std::vector<std::vector<cv::Point3f>> irObjP;   // Checkerboard world coordinates
  std::vector<std::vector<cv::Point3f>> rgbObjP;  // Checkerboard world coordinates

  // Define checkerboard Parameters for both frames
  float fieldSize = 0.040;
  int checkerBoard[2] = {9, 6};        // Checkerboard pattern
  cv::Size patternSize(9 - 1, 6 - 1);  // Number of inner corners per a chessboard row and column

  std::vector<std::vector<cv::Point2f>> rgbcorners(rgbFileNames.size());  // corners in rgb frames
  //std::vector<std::vector<cv::Point2f>> ircorners(irFileNames.size());    // corners in ir frames
  std::vector<cv::Point2f> rgbimgp;  // Define the rgb img point
  //std::vector<cv::Point2f> irimgp;   // Define the ir img point

  // Define the world coordinates for 3D points

  std::vector<cv::Point3f> rgbobjp;  // define the object point in 3D
  for (int i = 1; i < checkerBoard[1]; i++) {
    for (int j = 1; j < checkerBoard[0]; j++) {
      rgbobjp.push_back(cv::Point3f(j * fieldSize, i * fieldSize, 0));
    }
  }

  // Define the world coordinates for 3D points of the ir frame
  std::vector<cv::Point3f> irobjp;  // define the object point in 3D
  for (int i = 1; i < checkerBoard[1]; i++) {
    for (int j = 1; j < checkerBoard[0]; j++) {
      irobjp.push_back(cv::Point3f(j * fieldSize, i * fieldSize, 0));
    }
  }

  // Detect corners of the checkerboard in the RGB Frames
  cv::Mat rgbimg;
  std::size_t i2 = 0;
  std::size_t i = 0;
  std::size_t i_deleted = 0;
  for (auto const& f : rgbFileNames) {
    std::cout << std::string(f) << std::endl;
    rgbimg = cv::imread(f);  // Load the images
    cv::Mat rgbgray;         // grayscale the image
    cv::cvtColor(rgbimg, rgbgray, cv::COLOR_RGB2GRAY);
    bool rgbPatternFound = cv::findChessboardCorners(rgbgray, patternSize, rgbcorners[i2],
                                                     cv::CALIB_CB_ADAPTIVE_THRESH
                                                     //+ cv::CALIB_CB_NORMALIZE_IMAGE
                                                     + cv::CALIB_CB_FILTER_QUADS);
    // Use cv::cornerSubPix() to refine the found corner detections with default values given by opencv
    if (rgbPatternFound) {
      vector<Point_<float>> temp_ = rgbcorners[i2];
      cv::cornerSubPix(
          rgbgray, temp_, cv::Size(11, 11), cv::Size(-1, -1),// winsize 11,11 gute
          cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.01));
      rgbcorners[i2] = temp_;

      rgbObjP.push_back(rgbobjp);
      i2++;
    } else {
      rgbcorners.erase(next(rgbcorners.begin(), i - i_deleted));
      cout << "!!!!!!!!File " << rgbFileNames[i] << "not used!!!!!!!!" << endl;
      allJointStates.erase(next(allJointStates.begin(), i - i_deleted));
      cout << "!!!!!!!!Joint state " << rgbFileNames[i] << "not used!!!!!!!!" << endl;
      rgbFileNames.erase(next(rgbFileNames.begin(), i - i_deleted));
      cout << "!!!!!!!!Joint state " << rgbFileNames[i] << "not used!!!!!!!!" << endl;
      i_deleted++;
    }

    i++;
  }
  cout << "All RGB Corners detected and safed in rgbcorners\n";

  for(int i =0; i <  rgbFileNames.size() ; i++){
    // Display the detected pattern on the chessboard
    rgbimg = cv::imread(rgbFileNames[i]);
    cv::drawChessboardCorners(rgbimg, patternSize, rgbcorners[i], true);
    //cv::imshow("RGB chessboard corner detection", rgbimg);
    //cv::waitKey(0);
  }

  /*
  // Detect corners of the checkerboard in the IR Frames
  cv::Mat irimg;
  std::size_t m = 0;
  std::size_t m2 = 0;
  std::size_t m_deleted = 0;
  for (auto const& f : irFileNames) {
    std::cout << std::string(f) << std::endl;
    irimg = cv::imread(f);  // Load the images
    cv::Mat irgray;         // grayscale the image
    cv::cvtColor(irimg, irgray, cv::COLOR_RGB2GRAY);
    bool irpatternFound =
        cv::findChessboardCorners(irgray, patternSize, ircorners[m2],
                                  cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

    // Display the detected chessboard pattern on the ir frames
    // cv::drawChessboardCorners(irimg, patternSize, ircorners[m], irpatternFound);
    // cv::imshow("IR chessboard corner detection", irimg);
    // cv::waitKey(1);
    if (irpatternFound) {
      cv::cornerSubPix(
          irgray, ircorners[m2], cv::Size(20, 20), cv::Size(-1, -1),
          cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.0001));
      irObjP.push_back(irobjp);
      m2++;
    } else {
      ircorners.erase(next(ircorners.begin(), m - m_deleted));
      cout << endl << "!!!!!!!!File " << irFileNames[m] << "not used!!!!!!!!" << endl << endl;
      m_deleted++;
    }
    m++;
  }

  cout << "All IR Corners detected and safed in ircorners\n";
   */

  // Calibrate the rgb frame intrinsics
  cv::Mat rgbK;  // calibration Matrix K
  Mat rgbk = (Mat1d(1, 8));
  std::vector<cv::Mat> rvecs, tvecs;
  std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
  int flags = cv::CALIB_FIX_S1_S2_S3_S4 + cv::CALIB_FIX_TAUX_TAUY +
              cv::CALIB_RATIONAL_MODEL;  // used to compute k4-k6

  std::cout << "Calibrating rgb intrinsics...\n" << endl;

  float rgberror =
      cv::calibrateCamera(rgbObjP, rgbcorners, rgbFrameSize, rgbK, rgbk, rvecs, tvecs, flags);
  std::cout << "Reprojection error of rgb frames = " << rgberror << "\nK =\n"
            << rgbK << "\nk=\n"
            << rgbk << std::endl;

  cv::Mat rvec;
  cv::Mat tvec;
  std::cout << "rgbObjP.size: " << rgbObjP.size() << " rgbcorners.size: " << rgbcorners.size() << " rgbFileNames.size: " << rgbFileNames.size() << std::endl;
  for (int i = 0; i < rgbFileNames.size(); i++) {
    cv::InputArray rgb1ObjP = rgbObjP[i];
    cv::InputArray rgb1corners = rgbcorners[i];
    cv::solvePnPRansac(rgb1ObjP, rgb1corners, rgbK, rgbk, rvec, tvec);

    cout << "rvec camera" << rvec << endl;
    cout << "tvec camera" << tvec << endl;

    //Mat R;
    //Rodrigues(rvec, R);
    //R = R.t();
    //tvec = -R*tvec;
    //Rodrigues(R, rvec);
    cameraPosesR.push_back(rvec);
    cv::Mat temp;
    cv::Rodrigues(rvec, temp);
    temp = temp.t();
    R_target2cam.push_back(temp);
    t_target2cam.push_back(tvec);
    cout << "R_target2cam: " << temp << endl;
    cout << "t_target2cam: " << tvec << endl;
  }

/*


  // Calibrate the ir frame intrinsics
  cv::Mat irK;  // calibration Matrix K
  cv::Mat irk = (Mat1d(1, 8));
  std::vector<cv::Mat> irrvecs, irtvecs;
  std::vector<double> irIntrinsics, irExtrinsics, irperViewErrors;
  int irflags = cv::CALIB_FIX_S1_S2_S3_S4 + cv::CALIB_FIX_TAUX_TAUY +
                cv::CALIB_RATIONAL_MODEL;  // used to compute k4-k6
  std::cout << "Calibrating ir intrinsics...\n" << std::endl;
  // 4. Call "float error = cv::calibrateCamera()" with the input coordinates
  // and output parameters as declared above...
  float irerror =
      cv::calibrateCamera(irObjP, ircorners, irFrameSize, irK, irk, irrvecs, irtvecs, irflags);
  std::cout << "Reprojection error of the ir frames = " << irerror << "\nK =\n"
            << irK << "\nk=\n"
            << irk << std::endl;

  cv::Mat rgbmapX, rgbmapY;
  cv::initUndistortRectifyMap(rgbK, rgbk, cv::Matx33f::eye(), rgbK, rgbFrameSize, CV_32FC1, rgbmapX,
                              rgbmapY);
  cv::Mat irmapX, irmapY;
  cv::initUndistortRectifyMap(irK, irk, cv::Matx33f::eye(), irK, irFrameSize, CV_32FC1, irmapX,
                              irmapY);

  // Show lens corrected rgb images
  for (auto const& f : rgbFileNames) {
    std::cout << std::string(f) << std::endl;

    cv::Mat rgbimg = cv::imread(f, cv::IMREAD_COLOR);
    cv::Mat rgbimgUndistorted;
    // 5. Remap the image using the precomputed interpolation maps.
    cv::remap(rgbimg, rgbimgUndistorted, rgbmapX, rgbmapY, cv::INTER_LINEAR);
  }

  cv::Mat K1, K2, R, F, E;
  cv::Vec3d T;
  cv::Mat D1, D2;
  K1 = rgbK;
  K2 = irK;
  D1 = rgbk;
  D2 = irk;
  */

  /*
  float error = cv::calibrateCamera(Q, q, frameSize, K, k, rvecs, tvecs, flags);
  std::cout << "Reprojection error = " << error << "\nK =\n" << K << "\nk=\n" << k << std::endl;
  // Precompute lens correction interpolation
  cv::Mat mapX, mapY;
  cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1, mapX, mapY);
  return 0;
   */
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

void transform2rv(tf::StampedTransform transform, cv::Mat& rvec, cv::Mat& tvec){
  tvec = cv::Mat::zeros(3,1,CV_64F);
  rvec = cv::Mat::zeros(3,3,CV_64F);
  auto rot = transform.getBasis();
  for(int i = 0; i < 3; i++){
    for(int k = 0; k < 3; k++){
      rvec.at<double>(i, k) = rot[i][k];
    }
  }
  auto tran = transform.getOrigin();
  //quat2angaxis(rot.getX(), rot.y, rot.z, rot.w, rvec);
  tvec.at<double>(0,0) = tran.getX();
  tvec.at<double>(1,0) = tran.getY();
  tvec.at<double>(2,0) = tran.getZ();
  return;
}

// perform hand-eye calibration with the calculated transforms
void handEye(){

  vector<cv::Mat> R_gripper2base, t_gripper2base;
  for(int i=0; i<allJointStates.size(); i++)
  {
    cv::Mat R, t;
    transform2rv(allJointStates[i], R, t);
    cout << "R_gripper2base" << R <<endl;
    cout << "t_gripper2base " << t << endl;

    R_gripper2base.push_back(R);
    t_gripper2base.push_back(t);
  }
  cv::Mat R_cam2gripper, t_cam2gripper;

  ROS_INFO("Starting hand-eye calibration.");

  calibrateHandEye_(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper, t_cam2gripper);
  cout << "hand eye transformation: translation:\n" << t_cam2gripper << endl;
  cout << "hand eye transformation: rotation:\n" << R_cam2gripper << endl;

  // convert rotation matrix to angle axis
  Rodrigues(R_cam2gripper, R_cam2gripper);
  // convert angle axsi to quaternion
  vector<double> q;
  angaxis2quat(R_cam2gripper, q);

  // saving calibration results
  ofstream myfile;
  myfile.open(path + "/camera_hand_eye_calibration.yaml");
  myfile << "# This is an autogenerated file to store camera and hand-eye calibration results\n";
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
  tfListener = new tf::TransformListener();
  // callbacks. each image type has its own saving callback
  image_transport::Subscriber rgb_sub = it.subscribe("/calibration_rgb_img", 1, rgbImageWrite);
  //image_transport::Subscriber ir_sub = it.subscribe("/calibration_ir_img", 1, irImageWrite);
  ros::Subscriber joint_states_sub = nh.subscribe("/calibration_joint_states", 1, jointStatesWrite);

  std::cout << "\nready to receive frames" << endl;
  while(js_count < n_frames || rgb_count < n_frames) {
    ros::spinOnce();
  }
  cout << "\ncollected " << js_count << " frames" << endl;
  cout << "starting camera calibration" << endl;
  cameraCalibration();
  handEye();

  /* TODO: service that publishes matrix<xd> of defined size containing hand eye matrix*/

  return 0;
}
