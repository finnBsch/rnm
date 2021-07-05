// cameraCalibration: rotation matrix is pretty close to provided transofmrations,
// translation vector is very different
// TODO: fix cameraCalibration translation vector


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
//#include <geometry_msgs/TransformStamped.h>
//#include <tf/transform_listener.h>
#include <boost/filesystem.hpp>
#include <eigen3/Eigen/Dense>

#include <opencv2/core/mat.hpp>

//#include <tf2_ros/transform_listener.h>
#include "calibration/handEyeCalibration.h"


using namespace cv;
using namespace std;
using namespace Eigen;

//// settings:
// define number of frames to be extracted and used for calibration:
int n_frames = 40;

// only one of the following should be true, with exception of use_preset

// set true to write joint states and images to files:
bool write_files = false;
// set true to receive frames + joint states from get_images_node, and calibrate from those:
bool receive_frames = true; // <--- standard setting
// true to read from provided txt files:
bool from_folder = false;
// use written images for cameracal and joint_states.txt for handeye:
bool from_folder_ = false;

// using given k and K matrices:
bool use_preset = false;
// calibration data path:
string path = "/home/lars/cal_data";
//string path = "/home/rnm_grp1/rgb";


// counters for filenames
int pose_rgb = 1;
int pose_ir = 1;
int js_count = 0;
int rgb_count = 0;

// container for camera calibration data
//std::vector<cv::Mat> rvecs, tvecs;
std::vector<cv::Mat> cameraPosesR;
std::vector<cv::Mat> R_target2cam;
std::vector<cv::Mat> t_target2cam;

vector<vector<double>> allJointStates;
vector<array<cv::Mat,2>> allTransforms;

Mat rgbK;  // calibration Matrix K
Mat rgbk = (Mat1d(1, 8));

// helper function from forward_kinematics:
MatrixXd get_transformationmatrix(const float theta,
                                  const float a,
                                  const float d,
                                  const float alpha) {
  Matrix4d ret_mat;
  ret_mat << cos(theta), -sin(theta), 0, a, sin(theta) * cos(alpha), cos(theta) * cos(alpha),
      -sin(alpha), -d * sin(alpha), sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha),
      d * cos(alpha), 0, 0, 0, 1;
  return ret_mat;
}

// calculating transformation (pose) from joint angles,
// adapted version from forward_kinematics
array<Mat, 2> get_forward_kinematics_transformation(VectorXd a, vector<double> joint_angles_,
                                                    VectorXd d, VectorXd alpha) {
  array<MatrixXd, 8> a_;
  Matrix4d A_total;
  Mat one_R_gripper2base = Mat::zeros(3,3, CV_64F);
  Mat one_t_gripper2base = Mat::zeros(3,1,CV_64F);

  for (int i = 0; i < 7; i++) {
    a_.at(i) = get_transformationmatrix(joint_angles_[i], a(i), d(i), alpha(i));
  }
  a_.at(7) = get_transformationmatrix(0, a(7), d(7), alpha(7));
  A_total = a_.at(0) * a_.at(1) * a_.at(2) * a_.at(3) * a_.at(4) * a_.at(5) * a_.at(6) * a_.at(7);

  for (int i = 0; i < A_total.rows() - 1; i++) {
    for (int j = 0; j < A_total.cols() - 1; j++) {
      one_R_gripper2base.at<double>(i, j) = A_total(i, j);
    }
    one_t_gripper2base.at<double>(i) = A_total(i, 3);
  }

  return {one_R_gripper2base, one_t_gripper2base};
}

// saving jointState msg
void jointStatesWrite(const sensor_msgs::JointState& msg) {
  //// get joint angles
  if(js_count<n_frames) {
    //tf::StampedTransform joint_state;
    //tfListener->lookupTransform(robot_base, gripper, ros::Time(0), joint_state);

    vector<double> one_joint_states;
    one_joint_states = msg.position;
    allJointStates.push_back(one_joint_states);
    js_count++;


    cout << "one_joint_states: " << endl;
    cout << "[";
    for (int i=0; i < one_joint_states.size(); i++) {
      cout << one_joint_states[i] << ", ";
    }
    cout << "]" << endl;
  }
}
void jointStatesWriteFile(const sensor_msgs::JointState& msg) {
  string co = ",";
  ofstream foutput;
  ifstream finput;
  finput.open(path + "/joint_states.txt");
  foutput.open(path + "/joint_states.txt", ios::app);

  vector<double, allocator<void>::rebind<double>::other> position = msg.position;
  if (finput.is_open()) {
    foutput << position[0] << " " << position[1] << " " << position[2] << " " << position[3]
            << " " << position[4] << " " << position[5] << " " << position[6] << " "
            << endl;
    cout << "joint_states written to '" << path << "/joint_states.txt" << endl;
  }
  finput.close();
  foutput.close();
}

// ir image writing
void irImageWrite(const sensor_msgs::ImageConstPtr& msg) {
  try {
    // convert image from ros msg to bgr8 (cv)
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;


    // create filename
    stringstream ss;
    string name = "/imgs/pose_";
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
      string name = "/imgs/pose_";
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
  cv::Size rgbFrameSize(2048, 1536);
  //cv::Size irFrameSize(640, 576);

  std::vector<cv::String> rgbFileNames(n_frames);
  for (int i = 0; i < n_frames; i++) {
    rgbFileNames[i] = path + "/imgs/pose_" + to_string(i + 1) + ".jpg";
  }
  // std::vector<cv::String> irFileNames;
  //  std::string rgbFolder("/home/lars/CLionProjects/CameraCalibrationtests/cal_imgs/rgb/*.jpg");
  // std::string rgbFolder("/home/nico/catkin_ws/src/frame_reader/cal_imgs/new/rgb/*.jpg");
  // cv::glob(rgbFolder, rgbFileNames, true);  // load rgb images into opencv
  // std::string irFolder("/home/nico/catkin_ws/src/frame_reader/cal_imgs/new/ir/*.jpg");
  //  std::string irFolder("/home/lars/CLionProjects/CameraCalibrationtests/cal_imgs/ir/*.jpg");
  // cv::glob(irFolder, irFileNames, true);

  // std::vector<std::vector<cv::Point3f>> irObjP;   // Checkerboard world coordinates
  std::vector<std::vector<cv::Point3f>> rgbObjP;  // Checkerboard world coordinates

  // Define checkerboard Parameters for both frames
  float fieldSize = 0.040;
  int checkerBoard[2] = {9, 6};        // Checkerboard pattern
  cv::Size patternSize(9 - 1, 6 - 1);  // Number of inner corners per a chessboard row and column

  std::vector<std::vector<cv::Point2f>> rgbcorners(rgbFileNames.size());  // corners in rgb frames
  // std::vector<std::vector<cv::Point2f>> ircorners(irFileNames.size());    // corners in ir frames
  std::vector<cv::Point2f> rgbimgp;  // Define the rgb img point
  // std::vector<cv::Point2f> irimgp;   // Define the ir img point

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
  for(int i = 0; i < rgbFileNames.size(); i++){
    cout << i << " " << rgbFileNames[i] << endl;
  }
  for (int i = 0; i < rgbFileNames.size();) {
    string f = rgbFileNames[i];
    std::cout << std::string(f) << std::endl;
    rgbimg = cv::imread(f);  // Load the images
    cv::Mat rgbgray;         // grayscale the image
    cv::cvtColor(rgbimg, rgbgray, cv::COLOR_RGB2GRAY);
    bool rgbPatternFound = cv::findChessboardCorners(rgbgray, patternSize, rgbcorners[i],
                                                     cv::CALIB_CB_ADAPTIVE_THRESH
                                                     //+ cv::CALIB_CB_NORMALIZE_IMAGE
                                                     + cv::CALIB_CB_FILTER_QUADS);
    // Use cv::cornerSubPix() to refine the found corner detections with default values given by opencv
    if (rgbPatternFound) {
      vector<Point_<float>> temp_ = rgbcorners[i];
      cv::cornerSubPix(
          rgbgray, temp_, cv::Size(11, 11), cv::Size(-1, -1),  // winsize 11,11 gute
          cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.01));
      rgbcorners[i] = temp_;

      rgbObjP.push_back(rgbobjp);
      i++;
    } else {
      rgbcorners.erase(next(rgbcorners.begin(), i ));
      rgbFileNames.erase(next(rgbFileNames.begin(), i));
      allJointStates.erase(next(allJointStates.begin(), i ));
      cout << "!!!!!!!!File " << rgbFileNames[i] << "not used!!!!!!!!" << endl;
      i_deleted++;
    }

   // i++;
  }
  cout << "All RGB Corners detected and safed in rgbcorners\n";

  for (int i = 0; i < rgbFileNames.size(); i++) {
    // Display the detected pattern on the chessboard
    rgbimg = cv::imread(rgbFileNames[i]);
    cv::drawChessboardCorners(rgbimg, patternSize, rgbcorners[i], true);
    // cv::imshow("RGB chessboard corner detection", rgbimg);
    // cv::waitKey(0);
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
  //Mat rgbK;  // calibration Matrix K
  //Mat rgbk = (Mat1d(1, 8));
  std::vector<cv::Mat> rvecs, tvecs;
  std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
  int flags = cv::CALIB_FIX_S1_S2_S3_S4 + cv::CALIB_FIX_TAUX_TAUY +
              cv::CALIB_RATIONAL_MODEL;  // used to compute k4-k6
  //int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_RATIONAL_MODEL;
  // try these cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT

  std::cout << "Calibrating rgb intrinsics...\n" << endl;

  float rgberror =
      cv::calibrateCamera(rgbObjP, rgbcorners, rgbFrameSize, rgbK, rgbk, rvecs, tvecs, flags);
  std::cout << "Reprojection error of rgb frames = " << rgberror << "\nK =\n"
            << rgbK << "\nk=\n"
            << rgbk << std::endl;

  cv::Mat rvec;
  cv::Mat tvec;
  std::cout << "rgbObjP.size: " << rgbObjP.size() << " rgbcorners.size: " << rgbcorners.size() << " rgbFileNames.size: " << rgbFileNames.size() << std::endl;


  if (use_preset == 1) {
    float rgbKData[9] = {971.997, 0, 1022.58, 0, 971.906, 775.268, 0, 0, 1};
    float rgbkData[14] = {0.781141, -3.04307, 0.000614642, -2.35504e-05,
                          1.7224,   0.656092, -2.86714,    1.64986,
                          0,        0,        0,           0,
                          0,        0};  // k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4,tx,ty

    Mat rgbK = Mat(3, 3, CV_32F, rgbKData);
    Mat rgbk = Mat(1, 14, CV_32F, rgbkData);
    cout << "rgbk: " << rgbk << endl;
    cout << "rgbK: " << rgbK << endl;
  }




  for (int i = 0; i < rgbFileNames.size(); i++) {
    cv::InputArray rgb1ObjP = rgbObjP[i];
    cv::InputArray rgb1corners = rgbcorners[i];
    cv::solvePnPRansac(rgb1ObjP, rgb1corners, rgbK, rgbk, rvec, tvec);

    //Mat R;
    //Rodrigues(rvec, R);
    //R = R.t();
    //tvec = -R*tvec;
    //Rodrigues(R, rvec);
    //cameraPosesR.push_back(rvec);

    /////////////////////////////
    cv::Mat temp;
    cv::Rodrigues(rvec, temp);
    //temp = temp.t();
    R_target2cam.push_back(temp);
    cout << "R_target2cam: " << temp << endl;
    //tvec = -temp*tvec;
    t_target2cam.push_back(tvec);
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

/*
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
*/

void readJoint_States() {
  //// joint states
  try {
    ifstream file(path + "/joint_states.txt");
    string line;
    while (getline(file, line)) {
      vector<double> one_joint_states;
      istringstream stream(line);
      double d;
      while (stream >> d) {
        one_joint_states.push_back(d);
      }
      allJointStates.push_back(one_joint_states);
    }
    cout << "allJointStates.size: " << allJointStates.size() << endl;
    ROS_INFO("Done reading joint states.");
    cout << endl;
  }
  catch (...) {
    ROS_INFO("Error while reading joint states!!!");
    cout << endl;
  }
}

void readJointStates() {
  //// joint states
  try {
    ifstream file(path + "/joints.txt");
    string line;
    while (getline(file, line)) {
      vector<double> one_joint_states;
      istringstream stream(line);
      double d;
      while (stream >> d) {
        one_joint_states.push_back(d);
      }
      /*
      cout << "one_joint_states: " << endl;
      cout << "[";
      for (int i=0; i < one_joint_states.size(); i++) {
        cout << one_joint_states[i] << ", ";
      }
      cout << "]" << endl;
       */
      allJointStates.push_back(one_joint_states);
    }
    cout << "allJointStates.size: " << allJointStates.size() << endl;
    ROS_INFO("Done reading joint states.");
    cout << endl;
  }
  catch (...) {
    ROS_INFO("Error while reading joint states!!!");
    cout << endl;
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
    ifstream file(path + "/trk.txt");
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
      cout << "oneR_target2cam: " << endl << oneR_target2cam << endl;
      cout << "onet_target2cam: " << endl << onet_target2cam << endl;
      //////////
      Mat tempt;
      Mat tempR;
      oneR_target2cam = oneR_target2cam.t();
      onet_target2cam = -oneR_target2cam*onet_target2cam;
      t_target2cam.push_back(onet_target2cam);
      R_target2cam.push_back(oneR_target2cam);
      rot_count = 1;
    }
    cout << "R_target2cam.size: " << R_target2cam.size() << endl;
    cout << "t_target2cam.size: " << t_target2cam.size() << endl;
    ROS_INFO("Done reading checkerboard-to-camera-transformation.");
    cout << endl;

  }
  catch (...) {
    ROS_INFO("Error while reading checkerboard to camera transformation!!!");
  }
}



cv::Mat R_cam2gripper, t_cam2gripper;
// perform hand-eye calibration with the calculated transforms
void handEye(){
  static VectorXd a(8);
  static VectorXd d(8);
  static VectorXd alpha(8);
  a << 0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0;
  d << 0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107;
  alpha << 0, -M_PI/2, M_PI/2, M_PI/2, -M_PI/2, M_PI/2, M_PI/2, 0;
  vector<cv::Mat> R_gripper2base, t_gripper2base;


  for(int i=0; i<allJointStates.size(); i++)
  {
    //cv::Mat R, t;
    //transform2rv(allJointStates[i], R, t);


    auto fw_ret = get_forward_kinematics_transformation(a,allJointStates[i], d, alpha);
    auto R = fw_ret[0];
    auto t = fw_ret[1];

    cout << "R_gripper2base" << R <<endl;
    cout << "t_gripper2base " << t << endl;

    R_gripper2base.push_back(R);
    t_gripper2base.push_back(t);
  }




  ROS_INFO("Starting hand-eye calibration.");

  calibrateHandEye_(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper, t_cam2gripper);
  cout << "hand eye transformation: translation:\n" << t_cam2gripper << endl;
  cout << "hand eye transformation: rotation:\n" << R_cam2gripper << endl;

  // convert rotation matrix to angle axis
  Mat R_cam2grippervec;
  Rodrigues(R_cam2gripper, R_cam2grippervec);
  // convert angle axsi to quaternion
  vector<double> q;
  angaxis2quat(R_cam2grippervec, q);

  // saving calibration results
  ofstream myfile;
  myfile.open(path + "/camera_hand_eye_calibration.yaml");
  myfile << "# This is an autogenerated file to store camera and hand-eye calibration results\n";
  myfile << "camera_calibration:\n";
  myfile << "  # camera matrix\n";
  myfile << "  K: " << rgbK << endl;
  myfile << "  # distortion parameters\n";
  myfile << "  D: " << rgbk << endl;
  myfile << "hand_eye_position:\n";
  myfile << "  # rotaion matrix\n";
  //myfile << "  rotation: [" << q[0] << ", " << q[1] << ", ";
  myfile << "  rotation: [" << R_cam2gripper.at<double>(0,0) << ", " << R_cam2gripper.at<double>(0,1) << ", " << R_cam2gripper.at<double>(0,2) << ", "
      << R_cam2gripper.at<double>(1,0) << ", " << R_cam2gripper.at<double>(1,1) << ", " << R_cam2gripper.at<double>(2,2) << ", "
      << R_cam2gripper.at<double>(2,0) << ", " << R_cam2gripper.at<double>(2,1) << ", " << R_cam2gripper.at<double>(2,2) << "] ";
  //myfile << q[2] << ", " << q[3] << "]" << endl;
  myfile << "  # translation\n";
  myfile << "  translation: " << t_cam2gripper.reshape(1,1) << endl;
  ROS_INFO("calibration saved to 'camera_hand_eye_calibration.yaml");
}


bool handEyeOutput(calibration::handEyeCalibration::Request  &req,
                   calibration::handEyeCalibration::Response &res) {
  //float a = req.input; //dummy
  boost::array<double, 9> f_R_cam2gripper;
  boost::array<double, 3> f_t_cam2gripper;
  f_R_cam2gripper[0] = R_cam2gripper.at<double>(0, 0);
  f_R_cam2gripper[1] = R_cam2gripper.at<double>(0, 1);
  f_R_cam2gripper[2] = R_cam2gripper.at<double>(0, 2);
  f_R_cam2gripper[3] = R_cam2gripper.at<double>(1, 0);
  f_R_cam2gripper[4] = R_cam2gripper.at<double>(1, 1);
  f_R_cam2gripper[5] = R_cam2gripper.at<double>(1, 2);
  f_R_cam2gripper[6] = R_cam2gripper.at<double>(2, 0);
  f_R_cam2gripper[7] = R_cam2gripper.at<double>(2, 1);
  f_R_cam2gripper[8] = R_cam2gripper.at<double>(2, 2);

  f_t_cam2gripper[0] = t_cam2gripper.at<double>(0, 0);
  f_t_cam2gripper[1] = t_cam2gripper.at<double>(1, 0);
  f_t_cam2gripper[2] = t_cam2gripper.at<double>(2, 0);

  res.R_cam2gripper = f_R_cam2gripper;
  res.t_cam2gripper = f_t_cam2gripper;
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv,"calibration_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);


  //tfListener = new tf::TransformListener();
  if (write_files == true) {
    boost::filesystem::remove(path + "/joint_states.txt");
    boost::filesystem::remove_all(path + "/imgs");
    boost::filesystem::create_directory(path + "/imgs");
    image_transport::Subscriber rgb_sub = it.subscribe("/calibration_rgb_img", 1, rgbImageWrite);
    ros::Subscriber joint_states_sub = nh.subscribe("/calibration_joint_states", 1, jointStatesWriteFile);
    std::cout << "\nready to receive frames" << endl;
    while (js_count < n_frames || rgb_count < n_frames) {
      ros::spinOnce();
    }
    cout << "\ncollected " << js_count << " frames" << endl;
  }
  if (receive_frames == true) {
    boost::filesystem::remove(path + "/joint_states.txt");
    boost::filesystem::remove_all(path + "/imgs");
    boost::filesystem::create_directory(path + "/imgs");
    image_transport::Subscriber rgb_sub = it.subscribe("/calibration_rgb_img", 1, rgbImageWrite);
    //image_transport::Subscriber ir_sub = it.subscribe("/calibration_ir_img", 1, irImageWrite);
    ros::Subscriber joint_states_sub = nh.subscribe("/calibration_joint_states", 1, jointStatesWrite);
    std::cout << "\nready to receive frames" << endl;
    while (js_count < n_frames || rgb_count < n_frames) {
      ros::spinOnce();
    }
    cout << "\ncollected " << js_count << " frames" << endl;
    cout << "starting camera calibration" << endl;
    cameraCalibration();
  }
  if (from_folder == true) {
    readJointStates();
    // readEndPoses();
    readCheckPoses();
  }
  if (from_folder_ == true) {
    cout << "starting camera calibration" << endl;
    readJoint_States();
    cameraCalibration();
  }

  handEye();

  ros::ServiceServer service = nh.advertiseService("handEyeCalibration", handEyeOutput);

  /* TODO: service that publishes matrix<xd> of defined size containing hand eye matrix*/

  return 0;
}


