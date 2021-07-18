#include <iostream>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include "cv_handeye.h"
#include <boost/filesystem.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/mat.hpp>
#include "calibration/handEyeCalibration.h"
#include <cmath>


using namespace cv;
using namespace std;
using namespace Eigen;

//// settings:
// define number of frames to be received and used for calibration:
int n_frames = 40;

// only one of the following four bools should be true

// set true to write joint states and images to files, receiving them from get_images_node:
bool write_files = false; // can use files to calibrate
// set true to receive frames + joint states from get_images_node, and calibrate from those:
bool receive_frames = false; // does not save joint states permanently
// true to read from provided txt files:
bool from_folder = false;
// use written images for cameracal and joint_states.txt for handeye:
bool from_folder_ = true;

// performing ir calibration
bool irCalibration = true;
// performing stereo calibration
bool stereo = true;
// using given k and K matrices:
bool use_preset = false;

// hand-eye: ON/OFF , QR24
bool standardHandEye = false;
bool QR24 = true; // got us better results
// solvePnP method
string PnPMethod = "iterative"; // ransac or iterative, target2cam transforms created with iterative are closer to trk.txt

// calibration data path:
string path = "/home/nico/cal_data";
//string path = "/home/rnm_grp1/cal_data";

// Frame size
cv::Size rgbFrameSize(2048, 1536);
cv::Size irFrameSize(640, 576);
// Define checkerboard Parameters for both frames
float fieldSize = 0.040;
int checkerBoard[2] = {9, 6};        // Checkerboard pattern
cv::Size patternSize(9 - 1, 6 - 1);  // Number of inner corners per a chessboard row and column


// counters for filenames
int pose_rgb = 1;
int pose_ir = 1;
// counters to ensure correct assertion
int js_count = 0;
int rgb_count = 0;
int ir_count = 0;

// container for camera calibration data
std::vector<cv::Mat> R_target2cam; // cv input format
std::vector<cv::Mat> t_target2cam;
std::vector<cv::Mat> R_target2cam_og; // trk.txt format
std::vector<cv::Mat> t_target2cam_og;
Mat extrinsic = Mat::zeros(4,4,CV_64FC1);
Mat inv_extrinsic;
vector<vector<double>> allJointStates;

Mat rgbK;  // calibration Matrix K
Mat rgbk = (Mat1d(1, 8));
Mat irK;  // calibration Matrix K
Mat irk = (Mat1d(1, 8));

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

// calculating transformation (gripper2base) from joint angles,
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

// callback function, push jointState msg.position to allJointStates
void jointStatesWrite(const sensor_msgs::JointState& msg) {
  //// get joint angles
  if(js_count<n_frames) {
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

// callback function, writes jointState msg.position to txt file
void jointStatesWriteFile(const sensor_msgs::JointState& msg) {
  if (js_count < n_frames) {
    ofstream foutput;
    ifstream finput;
    finput.open(path + "/joint_states.txt");
    foutput.open(path + "/joint_states.txt", ios::app);

    vector<double, allocator<void>::rebind<double>::other> position = msg.position;
    if (finput.is_open()) {
      foutput << position[0] << " " << position[1] << " " << position[2] << " " << position[3]
              << " " << position[4] << " " << position[5] << " " << position[6] << " " << endl;
      cout << "joint_states [" << position[0] << " " << position[1] << " " << position[2] << " "
           << position[3] << " " << position[4] << " " << position[5] << " " << position[6]
           << "] written to '" << path << "/joint_states.txt'" << endl;
    }
    finput.close();
    foutput.close();
    js_count++;
  }
}

// callback function, writes ir image from msg to jpg file
void irImageWrite(const sensor_msgs::ImageConstPtr& msg) {
  try {
    if(ir_count < n_frames) {
      // convert image from ros msg to bgr8 (cv)
      cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

      // create filename
      stringstream ss;
      string name = "/imgs/ir/pose_";
      string type = ".jpg";
      ss << path << name << (pose_ir) << type;
      string filename = ss.str();
      ss.str("");
      // write img
      imwrite(filename, img);
      std::cout << "ir: Pose_" << pose_ir << " img written." << endl;
      pose_ir++;
      ir_count++;
    }
  }
  catch (cv_bridge::Exception &e) {
    std::cout << "Could not write ir: pose_" << pose_ir-1 << " image." << endl;
  }
}

// callback function, writes rgb image to jpg file
void rgbImageWrite(const sensor_msgs::ImageConstPtr& msg) {
  try {
    if(rgb_count < n_frames) {
      // convert image from ros msg to bgr8 (cv)
      cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

      // create filename
      stringstream ss;
      string name = "/imgs/rgb/pose_";
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

cv::Mat K1, K2, R, F, E;
cv::Vec3d T;
cv::Mat D1, D2;
double rms;
float rgberror;
float irerror;

// rgb, ir, stereo camera calibration; also computes target2cam transformations
int cameraCalibration() {
  std::vector<cv::String> rgbFileNames(n_frames);
  std::vector<cv::String> fileNames1(n_frames);
  std::vector<std::vector<cv::Point3f>> rgbObjP;  // Checkerboard world coordinates
  std::vector<std::vector<cv::Point2f>> rgbcorners;  // edges or vertices of rgb frames
  std::vector<cv::Point2f> rgbimgp;  // Define the rgb img point

  // vector of image file names from folder
  for (int i = 0; i < n_frames; i++) {
    rgbFileNames[i] = path + "/imgs/rgb/pose_" + to_string(i + 1) + ".jpg";
  }
  fileNames1 = rgbFileNames;
  // Define the world coordinates for 3D points of the rgb frame
  std::vector<cv::Point3f> rgbobjp;  // define the object point in 3D
  for (int i = 1; i < checkerBoard[1]; i++) {
    for (int j = 1; j < checkerBoard[0]; j++) {
      rgbobjp.push_back(cv::Point3f(j * fieldSize, i * fieldSize, 0));
    }
  }
  // Detect edges of the checkerboard in the RGB Frames
  cv::Mat rgbimg;
  std::size_t i_deleted = 0;
  for (int i = 0; i < rgbFileNames.size(); i++) {
    //cout << "rgb: " << i << " " << rgbFileNames[i] << endl;
  }
  for (int i = 0; i < rgbFileNames.size();) {
    string f = rgbFileNames[i];
    std::cout << std::string(f) << std::endl;
    rgbimg = cv::imread(f); // Load the images
    cv::Mat rgbgray; // grayscale the image
    cv::cvtColor(rgbimg, rgbgray, cv::COLOR_RGB2GRAY);
    vector<Point_<float>> rgb_corners_dummy;
    bool rgbPatternFound = cv::findChessboardCorners(rgbgray, patternSize, rgb_corners_dummy,
                                                     cv::CALIB_CB_ADAPTIVE_THRESH
                                                     //+ cv::CALIB_CB_NORMALIZE_IMAGE
                                                     + cv::CALIB_CB_FILTER_QUADS);
    // Use cv::cornerSubPix() to refine the found corner detections with values given by opencv
    // very sensitive to changes in winSize
    if (rgbPatternFound) {
      cv::cornerSubPix(
          rgbgray, rgb_corners_dummy, cv::Size(16, 16), cv::Size(-1, -1),
          cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.001));
      rgbcorners.push_back(rgb_corners_dummy);
      rgbObjP.push_back(rgbobjp);
      i++;
      // if can't detect all edges, delete filename from vector of filenames and associated joint states from txt file
    } else {
      cout << "File " << rgbFileNames[i] << " not used!" << endl;
      rgbFileNames.erase(next(rgbFileNames.begin(), i));
      allJointStates.erase(next(allJointStates.begin(), i));
      i_deleted++;
    }
  }
  cout << "All RGB Corners detected and saved in rgbcorners\n";

  //  // Display the detected pattern on the chessboard
  //  for (int i = 0; i < rgbFileNames.size(); i++) {
  //    rgbimg = cv::imread(rgbFileNames[i]);
  //    cv::drawChessboardCorners(rgbimg, patternSize, rgbcorners[i], true);
  //   cv::imshow("RGB chessboard corner detection", rgbimg);
  //     cv::waitKey(0);
  //  }
  //////////////////////////////////////////////
  // IR corner detection
  // same procedure
  std::vector<cv::String> irFileNames(n_frames);
  std::vector<cv::String> fileNames2(n_frames);

  std::vector<std::vector<cv::Point3f>> irObjP;  // Checkerboard world coordinates
  std::vector<std::vector<cv::Point2f>> ircorners;  // corners in ir frames
  std::vector<cv::Point2f> irimgp; // Define the ir img point

  if (irCalibration == true) {
    for (int i = 0; i < n_frames; i++) {
      irFileNames[i] = path + "/imgs/ir/pose_" + to_string(i + 1) + ".jpg";
    }
    fileNames2 = irFileNames;
    // Define the world coordinates for 3D points of the ir frame
    std::vector<cv::Point3f> irobjp;  // define the object point in 3D
    for (int i = 1; i < checkerBoard[1]; i++) {
      for (int j = 1; j < checkerBoard[0]; j++) {
        irobjp.push_back(cv::Point3f(j * fieldSize, i * fieldSize, 0));
      }
    }

    cv::Mat irimg;
    std::size_t i_deleted = 0;
    for (int i = 0; i < irFileNames.size(); i++) {
      //cout << "ir: " << i << " " << irFileNames[i] << endl;
    }
    for (int i = 0; i < irFileNames.size();) {
      string g = irFileNames[i];
      std::cout << std::string(g) << std::endl;
      irimg = cv::imread(g);  // Load the images
      cv::Mat irgray;         // grayscale the image
      cv::cvtColor(irimg, irgray, cv::COLOR_RGB2GRAY);
      vector<Point_<float>> ircorners_dummy;
      bool irpatternFound =
          cv::findChessboardCorners(irgray, patternSize, ircorners_dummy,
                                    cv::CALIB_CB_ADAPTIVE_THRESH
              //+ cv::CALIB_CB_NORMALIZE_IMAGE
          );

      if (irpatternFound) {
        cv::cornerSubPix(
            irgray, ircorners_dummy, cv::Size(18, 18), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.001));
        ircorners.push_back(ircorners_dummy);
        irObjP.push_back(irobjp);
        i++;
      } else {
        cout << "File " << irFileNames[i] << " not used!" << endl;
        irFileNames.erase(next(irFileNames.begin(), i));
        i_deleted++;
      }
    }

    cout << "All IR Corners detected and saved in ircorners\n";
    // Display the detected pattern on the chessboard
    /*for (int i = 0; i < irFileNames.size(); i++) {
      irimg = cv::imread(irFileNames[i]);
      cv::drawChessboardCorners(irimg, patternSize, ircorners[i], true);
     cv::imshow("IR chessboard corner detection", irimg);
       cv::waitKey(0);
    }*/
  }

  // Calibrate the rgb frame intrinsics
  std::vector<cv::Mat> rvecs, tvecs;
  std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
  // cv flags for calibrateCamera
  int flags = CALIB_RATIONAL_MODEL;
  //  try these
  // cv::CALIB_FIX_ASPECT_RATIO fixes fx, fx/fy ratio stays the same
  // cv::CALIB_ZERO_TANGENT_DIST p1,p2 =0 and stay 0
  // cv::CALIB_FIX_K1 ...K6 fixes k1-k6 separately (radial distortion)
  // cv::CALIB_RATIONAL_MODEL enables k4,k5,k6
  // cv::CALIB_THIN_PRISM_MODEL enables s1,s2,s3,s4
  // cv::CALIB_FIX_S1_S2_S3_S4
  // cv::CALIB_TILTED_MODEL enables taux, tauy

  // order of dist coeffs: k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4,tx,ty

  std::cout << "Calibrating rgb intrinsics...\n" << endl;

  // actual calibration and error
  rgberror =
      cv::calibrateCamera(rgbObjP, rgbcorners, rgbFrameSize, rgbK, rgbk, rvecs, tvecs, flags);
  std::cout << "Reprojection error of rgb frames = " << rgberror << "\nK =\n"
            << rgbK << "\nk=\n"
            << rgbk << std::endl;

  std::cout << "rgbObjP.size: " << rgbObjP.size() << " rgbcorners.size: " << rgbcorners.size()
            << " rgbFileNames.size: " << rgbFileNames.size() << std::endl;

  // values from given txt file, only used for comparison
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

  /////////////////////////////////////////////
  // computing target2cam for each frame
  for (int i = 0; i < rgbFileNames.size(); i++) {
    Mat rvec;
    Mat tvec;
    cv::InputArray rgb1ObjP = rgbObjP[i];
    cv::InputArray rgb1corners = rgbcorners[i];
    // solvePnPRansac computes transformations from target (checkerboard) to camera
    // requires data from camera calibration
    if (PnPMethod == "ransac") {
      cv::solvePnPRansac(rgb1ObjP, rgb1corners, rgbK, rgbk, rvec, tvec);
    }
    if (PnPMethod == "iterative") {
      cv::solvePnP(rgb1ObjP, rgb1corners, rgbK, rgbk, rvec, tvec, SOLVEPNP_ITERATIVE);
    }

    // push back each transformation to target2cam
    cv::Mat temp;
    cv::Rodrigues(rvec, temp);
    R_target2cam.push_back(temp);
    t_target2cam.push_back(tvec);
    //cout << "t_target2cam: " << tvec << endl;
    //cout << "R_target2cam: " << temp << endl;

    //// trk.txt format
    // format ofthe supplied txt file, used for comparison
    Mat tempR = temp.t();
    Mat tempt = -tempR * tvec;
    R_target2cam_og.push_back(tempR);
    t_target2cam_og.push_back(tempt);
    //cout << "R_target2cam_og: " << endl << tempR << endl;
    //cout << "t_target2cam_og: " << endl << tempt << endl;
  }

  // ir calibration using edges and object points defined earlier
  if (irCalibration == true) {
    // Calibrate the ir frame intrinsics
    std::vector<cv::Mat> irrvecs, irtvecs;
    std::vector<double> irIntrinsics, irExtrinsics, irperViewErrors;
    int irflags = cv::CALIB_FIX_S1_S2_S3_S4 + cv::CALIB_FIX_TAUX_TAUY +
                  cv::CALIB_RATIONAL_MODEL;  // used to compute k4-k6
    std::cout << "Calibrating ir intrinsics...\n" << std::endl;

    irerror =
        cv::calibrateCamera(irObjP, ircorners, irFrameSize, irK, irk, irrvecs, irtvecs, irflags);
    std::cout << "Reprojection error of the ir frames = " << irerror << "\nK =\n"
              << irK << "\nk=\n"
              << irk << std::endl;

    std::cout << "irObjP.size: " << irObjP.size() << " ircorners.size: " << ircorners.size()
              << " irFileNames.size: " << irFileNames.size() << std::endl;
    /*
    cv::Mat rgbmapX, rgbmapY;
    cv::initUndistortRectifyMap(rgbK, rgbk, cv::Matx33f::eye(), rgbK, rgbFrameSize, CV_32FC1,
                                rgbmapX, rgbmapY);
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
    */
  }

  ///////////////////////////////////////////////////////
  // Perform the stereo calibration
  if (stereo == true) {
    vector<vector<Point3f>> object_points;
    vector<vector<Point2f>> imagePoints1, imagePoints2;
    vector<Point2f> corners1, corners2;
    vector<vector<Point2f>> left_img_points, right_img_points;
    Mat img1, img2;
    int num_deleted = 0;
    int n = 1;
    for (int i = 0; i < n_frames; i++) {
      img1 = cv::imread(fileNames1[i]);  // Load the rgb images
      img2 = cv::imread(fileNames2[i]);   // Load the ir images
      cv::Mat gray1;                       // grayscale the rgb image
      cv::cvtColor(img1, gray1, cv::COLOR_RGB2GRAY);
      cv::Mat gray2;  // grayscale the ir image
      cv::cvtColor(img2, gray2, cv::COLOR_RGB2GRAY);

      bool found1 = cv::findChessboardCorners(gray1, patternSize, corners1,
                                              CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_FILTER_QUADS);

      bool found2 = cv::findChessboardCorners(gray2, patternSize, corners2,
                                              CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);

      vector<Point3f> obj;
      for (int j = 1; j < checkerBoard[1]; j++) {
        for (int k = 1; k < checkerBoard[0]; k++) {
          obj.push_back(Point3f((float)k * fieldSize, (float)j * fieldSize, 0));
        }
      }

      if (found1 && found2) {
        // Refine CornerDetection
        cv::cornerSubPix(
            gray1, corners1, cv::Size(16, 16), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.001));

        cv::drawChessboardCorners(gray1, patternSize, corners1, found1);

        cv::cornerSubPix(
            gray2, corners2, cv::Size(18, 18), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.001));

        cv::drawChessboardCorners(gray2, patternSize, corners2, found2);
        cout << n << " good framepairs" << endl;
        imagePoints1.push_back(corners1);
        imagePoints2.push_back(corners2);
        object_points.push_back(obj);
        cout << "leftImg: " << fileNames1[i] << " and rightImg: " << fileNames2[i] << endl;
        n++;
      } else {
        cout << "Pairs " << fileNames2[i] << "not used!" << endl;
      }
    }


    for (int i = 0; i < imagePoints1.size(); i++) {
      vector<Point2f> v1, v2;
      for (int j = 0; j < imagePoints1[i].size(); j++) {
        v1.push_back(Point2f((double)imagePoints1[i][j].x, (double)imagePoints1[i][j].y));
        v2.push_back(Point2f((double)imagePoints2[i][j].x, (double)imagePoints2[i][j].y));
      }
      left_img_points.push_back(imagePoints1[i]);
      right_img_points.push_back(imagePoints2[i]);
    }

    std::cout << "Starting Calibration\n";
    K1 = rgbK;
    K2 = irK;
    D1 = rgbk;
    D2 = irk;

    int stereoflags =
        CALIB_FIX_INTRINSIC + CALIB_FIX_S1_S2_S3_S4 + CALIB_FIX_TAUX_TAUY + CALIB_RATIONAL_MODEL;

    std::cout << "Compute stereocalibration...\n";
    rms = stereoCalibrate(object_points, left_img_points, right_img_points, K1, D1, K2, D2,
                          rgbFrameSize, R, T, E, F, stereoflags);
    cout << "Done with RMS error=" << rms << endl;

    std::cout << "object_points.size: " << object_points.size() << " left_img_points.size: " << left_img_points.size()
              << " right_img_points.size: " << right_img_points.size() << std::endl;

    std::cout << "\n RGB Intrinsics after StereoCalibration "<< K1 << endl << D1 << endl;
    std::cout << "\n IR Intrinsics after StereoCalibration " << K2 << endl << D2 << endl;

    // Precompute lens correction interpolation
    cout << "Done Stereocalibration\n";

    // Rectification
    cout << "Starting Rectification\n";
    cv::Mat R1, R2, P1, P2, Q;
    stereoRectify(K1, D1, K2, D2, irFrameSize, R, T, R1, R2, P1, P2, Q);
    printf("Done Rectification\n");
  }
}

// conversion function from rotation matrix to angle axis
void matrix2angaxis(Mat R, Mat& angaxis) {
  // R: 3x3
  // no check for identity matrix implemented, this only works for angles other than 0 and 180
  double angle = acos((R.at<double>(0,0)+R.at<double>(1,1)+R.at<double>(2,2)-1)/2);

  double a = pow(R.at<double>(2,1)-R.at<double>(1,2),2);
  double b = pow(R.at<double>(0,2)-R.at<double>(2,0),2);
  double c = pow(R.at<double>(1,0)-R.at<double>(0,1),2);

  double x = (R.at<double>(2,1)-R.at<double>(1,2))/sqrt(a+b+c);
  double y = (R.at<double>(0,2)-R.at<double>(2,0))/sqrt(a+b+c);
  double z = (R.at<double>(1,0)-R.at<double>(0,1))/sqrt(a+b+c);

  angaxis = Mat::zeros(1,4,CV_64FC1);
  angaxis.at<double>(0,0) = x;
  angaxis.at<double>(0,1) = y;
  angaxis.at<double>(0,2) = z;
  angaxis.at<double>(0,3) = angle;
}

// function to read joint states from own txt file
void readJoint_States() {
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

// same function to read joint states from provided text file
void readJointStates() {
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

// read checkerboard poses (target2cam transformations) from provided txt file
void readCheckPoses() {
  // reading: 40 lines, each line consists of 16 entries (homogeneous matrix)
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
      // original transformations
      oneR_target2cam = oneR_target2cam.reshape(1, 3);
      //cout << "oneR_target2cam: " << endl << oneR_target2cam << endl;
      //cout << "onet_target2cam: " << endl << onet_target2cam << endl;
      R_target2cam_og.push_back(oneR_target2cam);
      t_target2cam_og.push_back(onet_target2cam);

      // flip R and rotate t to cv::calibrateHandEye input format
      Mat tempt;
      Mat tempR;
      tempR = oneR_target2cam.t();
      tempt = -tempR * onet_target2cam;
      t_target2cam.push_back(tempt);
      R_target2cam.push_back(tempR);
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
vector<cv::Mat> R_gripper2base, t_gripper2base;
// perform hand-eye calibration with the calculated transforms
// tsai lenz
void handEye(){
  // some values needed to calculate gripper2base
  static VectorXd a(8);
  static VectorXd d(8);
  static VectorXd alpha(8);
  a << 0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0;
  d << 0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107;
  alpha << 0, -M_PI/2, M_PI/2, M_PI/2, -M_PI/2, M_PI/2, M_PI/2, 0;

  // calculate gripper2base
  for(int i=0; i<allJointStates.size(); i++)
  {
    auto fw_ret = get_forward_kinematics_transformation(a,allJointStates[i], d, alpha);
    auto R = fw_ret[0];
    auto t = fw_ret[1];

    //cout << "R_gripper2base" << R <<endl;
    //cout << "t_gripper2base " << t << endl;

    R_gripper2base.push_back(R);
    t_gripper2base.push_back(t);
  }

  ROS_INFO("Starting Tsai-Lenz hand-eye calibration.");
  calibrateHandEye_(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper, t_cam2gripper);
  cout << "hand eye transformation: translation:\n" << t_cam2gripper << endl;
  cout << "hand eye transformation: rotation:\n" << R_cam2gripper << endl;
}

Mat R_target2base, t_target2base;
void handEyeQR24() {
  // needs: gripper2base, target2cam

  // fill gripper2base from joint states
  static VectorXd a(8);
  static VectorXd d(8);
  static VectorXd alpha(8);
  a << 0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0;
  d << 0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107;
  alpha << 0, -M_PI/2, M_PI/2, M_PI/2, -M_PI/2, M_PI/2, M_PI/2, 0;
  for(int i=0; i<allJointStates.size(); i++)
  {
    auto fw_ret = get_forward_kinematics_transformation(a,allJointStates[i], d, alpha);
    auto R = fw_ret[0];
    auto t = fw_ret[1];

    //cout << "R_gripper2base" << R <<endl;
    //cout << "t_gripper2base " << t << endl;

    R_gripper2base.push_back(R);
    t_gripper2base.push_back(t);
  }

  cout << endl << "Starting QR24 hand-eye calibration ..." << endl;

  // input
  Mat A(12*R_target2cam.size(), 24, CV_64FC1);
  Mat B(12*R_target2cam.size(), 1,CV_64FC1);

  //Mat zeros91 = Mat::zeros(9,1,CV_64FC1);
  Mat zeros91 = Mat::eye(9,1,CV_64FC1);
  Mat zeros93 = Mat::zeros(9,3,CV_64FC1);
  Mat negEye = -1 * Mat::eye(12,12,CV_64FC1);

  // output
  Mat X;

  // fill coefficent-matrices using our checkerboard poses (target2cam) and end-effector poses (gripper2base)
  for (int i = 0; i < R_target2cam.size(); i++) {

    Mat a00 = R_gripper2base[i] * R_target2cam[i].at<double>(0,0);
    Mat a01 = R_gripper2base[i] * R_target2cam[i].at<double>(1,0);
    Mat a02 = R_gripper2base[i] * R_target2cam[i].at<double>(2,0);
    Mat a10 = R_gripper2base[i] * R_target2cam[i].at<double>(0,1);
    Mat a11 = R_gripper2base[i] * R_target2cam[i].at<double>(1,1);
    Mat a12 = R_gripper2base[i] * R_target2cam[i].at<double>(2,1);
    Mat a20 = R_gripper2base[i] * R_target2cam[i].at<double>(0,2);
    Mat a21 = R_gripper2base[i] * R_target2cam[i].at<double>(1,2);
    Mat a22 = R_gripper2base[i] * R_target2cam[i].at<double>(2,2);
    Mat a30 = R_gripper2base[i] * t_target2cam[i].at<double>(0,0);
    Mat a31 = R_gripper2base[i] * t_target2cam[i].at<double>(1,0);
    Mat a32 = R_gripper2base[i] * t_target2cam[i].at<double>(2,0);

    // !!! Rect(x coord of top left corner, y coord of top left corner, width, height) creates rectangle
    // filling columns 1 to 9 (of 24)
    // creates 12rows by 9cols blocks per iteration, next iteration's data uses next 12 rows
    a00.copyTo(A(Rect(0,i*12,3,3)));
    a01.copyTo(A(Rect(3,i*12,3,3)));
    a02.copyTo(A(Rect(6,i*12,3,3)));
    a10.copyTo(A(Rect(0,3 + i*12,3,3)));
    a11.copyTo(A(Rect(3,3 + i*12,3,3)));
    a12.copyTo(A(Rect(6,3 + i*12,3,3)));
    a20.copyTo(A(Rect(0,6 + i*12,3,3)));
    a21.copyTo(A(Rect(3,6 + i*12,3,3)));
    a22.copyTo(A(Rect(6,6 + i*12,3,3)));
    a30.copyTo(A(Rect(0,9 + i*12,3,3)));
    a31.copyTo(A(Rect(3,9 + i*12,3,3)));
    a32.copyTo(A(Rect(6,9 + i*12,3,3)));

    // filling columns 10, 11, 12
    zeros93.copyTo(A(Rect(9, i*12,3,9)));
    R_gripper2base[i].copyTo(A(Rect(9,9 + i*12,3,3)));
    // filling columns 13 to 24
    negEye.copyTo(A(Rect(12,i*12,12,12)));

    // B:
    // 9rows by 1col blocks (of 12 rows)
    zeros91.copyTo(B(Rect(0,i*12,1,9)));
    // negative translation into row 10,11,12
    Mat negT = -1 * t_gripper2base[i];
    negT.copyTo(B(Rect(0,9 + i*12,1,3)));
  }

  // cv::solve(lhs,rhs,out,flag)
  // lhs is a*a, rhs a*1, out a*1, need 12 output values
  // out = argminx(src1*x - src2)
  solve(A,B,X,DECOMP_QR);

  // reshape, cam2gripper
  Mat R = X(Rect(0,0,1,9));
  Mat t = X(Rect(0,9,1,3));
  R_cam2gripper = R.reshape(0,3);
  R_cam2gripper = R_cam2gripper.t();
  t_cam2gripper = t;
  // target2base?
  Mat R2 = X(Rect(0,12,1,9));
  Mat t2 = X(Rect(0,21,1,3));
  R_target2base = R2.reshape(0,3);
  R_target2base = R_target2base.t();
  t_target2base = t2;

  cout << "t_cam2gripper: " << endl << t_cam2gripper << endl;
  cout << "R_cam2gripper: " << endl << R_cam2gripper << endl;
}

// service intended to supply hand-eye results
bool handEyeOutput(calibration::handEyeCalibration::Request  &req,
                   calibration::handEyeCalibration::Response &res) {

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

// calculate error of hand-eye transformation
float avg_trans_err;
double avg_rot_err;
void handEyeError(){
  int n_poses = R_target2cam.size();

  // loop:
  // A = base2target * gripper2base * cam2gripper * target2cam

  Mat A;

  // create homogeneous matrices
  Mat base2target = Mat::zeros(4,4,CV_64FC1);
  Mat gripper2base = Mat::zeros(4,4,CV_64FC1);
  Mat cam2gripper = Mat::zeros(4,4,CV_64FC1);
  Mat target2cam = Mat::zeros(4,4,CV_64FC1);
  base2target.at<double>(3,3) = 1.0;
  gripper2base.at<double>(3,3) = 1.0;
  cam2gripper.at<double>(3,3) = 1.0;
  target2cam.at<double>(3,3) = 1.0;

  // base2target
  R_target2base.copyTo(base2target(Rect(0,0,3,3)));
  t_target2base.copyTo(base2target(Rect(3,0,1,3)));
  base2target = homogeneousInverse(base2target);

  // cam2gripper
  R_cam2gripper.copyTo(cam2gripper(Rect(0, 0, 3, 3)));
  t_cam2gripper.copyTo(cam2gripper(Rect(3, 0, 1, 3)));

  for (int pose = 0; pose < n_poses; pose++) {
    //    target2cam
    R_target2cam[pose].copyTo(target2cam(Rect(0, 0, 3, 3)));
    t_target2cam[pose].copyTo(target2cam(Rect(3, 0, 1, 3)));

    // gripper2base
    R_gripper2base[pose].copyTo(gripper2base(Rect(0, 0, 3, 3)));
    t_gripper2base[pose].copyTo(gripper2base(Rect(3, 0, 1, 3)));

    // A
    A = base2target * gripper2base * cam2gripper * target2cam;

    float trans_err =
        sqrt(pow(A.at<double>(0, 3), 2) + pow(A.at<double>(1, 3), 2) + pow(A.at<double>(2, 3), 2));
    //cout << endl << "A: " << endl << A << endl;
    //cout << "trans_err: " << trans_err << endl;
    avg_trans_err += trans_err;

    // rotational error, contains a mistake, entry A(0,0) is too big, arccos cannot be calculated
    Mat rot;
    rot = A(Rect(0,0,3,3));
    //cout << "rot: " << rot << endl;

    Mat angaxis_A;
    matrix2angaxis(rot, angaxis_A);
    //cout << "angaxis_A: " << angaxis_A << endl;
    avg_rot_err += angaxis_A.at<double>(0,3);
  }
  avg_trans_err = avg_trans_err / n_poses;
  cout << "avg trans_err: " << avg_trans_err << endl;

  // rotational error
  avg_rot_err = avg_rot_err / n_poses;
  //cout << "avg_rot_err: " << avg_rot_err << endl;
}

// write results to yaml files
// hand_eye_calibration.yaml contains hand eye results and a few settings, used to compare results
// camera_calibration.yaml contains the data that needs to replace the factory calibration of the azure kinect
void writeResults(string method) {
  if (from_folder == false && stereo == true) {
    // extrinsic camera matrix
    extrinsic.at<double>(3, 3) = float(1);
    Mat extrinsic_T = Mat::zeros(3, 1, CV_64FC1);
    extrinsic_T.at<double>(0, 0) = T[0];
    extrinsic_T.at<double>(1, 0) = T[1];
    extrinsic_T.at<double>(2, 0) = T[2];
    R.copyTo(extrinsic(Rect(0, 0, 3, 3)));
    extrinsic_T.copyTo(extrinsic(Rect(3, 0, 1, 3)));
    inv_extrinsic = homogeneousInverse(extrinsic);
  }

  // saving calibration results
  ofstream myfile;
  string filename = "/hand_eye_calibration.yaml";
  myfile.open(path + filename);
  myfile << "# Calibration data" << endl;
  myfile << "settings: " << endl;
  myfile << "from_folder=" << from_folder << ", " << "use_preset=" << use_preset << ", "
         << "hand-eye-method=" << method << endl;
  myfile << "camera_calibration:" << endl;
  myfile << "  # camera matrix" << endl;
  myfile << "  K: " << rgbK << endl;
  myfile << "  # distortion parameters" << endl;
  myfile << "  D: " << rgbk << endl;
  myfile << "hand-eye transformation:" << endl;
  myfile << "  # rotation matrix:" << endl;
  myfile << R_cam2gripper << endl;
  myfile << "  # translation vector:" << endl;
  myfile << t_cam2gripper << endl;
  myfile << "average translational error: " << avg_trans_err << endl;
  //myfile << "average rotational error: " << avg_rot_err << endl;
  myfile.close();

  string filename1 = "/camera_calibration.yaml";
  if (stereo == true) {
    ofstream myfile1;
    myfile1.open(path + filename1);
    myfile1 << "extrinsics:" << endl;
    myfile1 << "  depth_to_rgb:" << endl
            << "  -" << extrinsic.row(0) << endl
            << "  -" << extrinsic.row(1) << endl
            << "  -" << extrinsic.row(2) << endl;
    myfile1 << "  rgb_to_depth:" << endl
            << "  -" << inv_extrinsic.row(0) << endl
            << "  -" << inv_extrinsic.row(1) << endl
            << "  -" << inv_extrinsic.row(2) << endl;
    myfile1 << "  rms: " << rms << endl;
    myfile1 << "intrinsic_color:" << endl;
    myfile1 << "  camera_matrix:" << endl
            << "  -" << K1.row(0) << endl
            << "  -" << K1.row(1) << endl
            << "  -" << K1.row(2) << endl;
    myfile1 << "  k1: " << D1.at<double>(0) << endl;
    myfile1 << "  k2: " << D1.at<double>(1) << endl;
    myfile1 << "  k3: " << D1.at<double>(4) << endl;
    myfile1 << "  k4: " << D1.at<double>(5) << endl;
    myfile1 << "  k5: " << D1.at<double>(6) << endl;
    myfile1 << "  k6: " << D1.at<double>(7) << endl;
    myfile1 << "  p1: " << D1.at<double>(2) << endl;
    myfile1 << "  p2: " << D1.at<double>(3) << endl;
    myfile1 << "  rms: " << rgberror << endl;
    myfile1 << "intrinsic_depth:" << endl;
    myfile1 << " camera_matrix:" << endl
            << "  -" << K2.row(0) << endl
            << "  -" << K2.row(1) << endl
            << "  -" << K2.row(2) << endl;
    myfile1 << "  k1: " << D2.at<double>(0) << endl;
    myfile1 << "  k2: " << D2.at<double>(1) << endl;
    myfile1 << "  k3: " << D2.at<double>(4) << endl;
    myfile1 << "  k4: " << D2.at<double>(5) << endl;
    myfile1 << "  k5: " << D2.at<double>(6) << endl;
    myfile1 << "  k6: " << D2.at<double>(7) << endl;
    myfile1 << "  p1: " << D2.at<double>(2) << endl;
    myfile1 << "  p2: " << D2.at<double>(3) << endl;
    myfile1 << "  rms: " << irerror << endl;
    myfile1.close();
  }

  ROS_INFO("calibration saved to ");
  cout << path << filename << endl;
  if (stereo == true) {
    cout << path << filename1 << endl;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv,"calibration_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  if (write_files == true) {
    boost::filesystem::remove(path + "/joint_states.txt");
    boost::filesystem::remove_all(path + "/imgs/rgb");
    boost::filesystem::remove_all(path + "/imgs/ir");
    boost::filesystem::create_directory(path + "/imgs");
    boost::filesystem::create_directory(path + "/imgs/rgb");
    boost::filesystem::create_directory(path + "/imgs/ir");
    boost::filesystem::ofstream(path + "/joint_states.txt");
    // subscribers to topics that are being published by get_images_node ( publishes most recent frames and joint states
    image_transport::Subscriber rgb_sub = it.subscribe("/calibration_rgb_img", 1, rgbImageWrite);
    image_transport::Subscriber ir_sub = it.subscribe("/calibration_ir_img", 1, irImageWrite);
    ros::Subscriber joint_states_sub = nh.subscribe("/calibration_joint_states", 1, jointStatesWriteFile);
    std::cout << "\nready to receive frames" << endl;
    // only spin while counter has not been reached
    while (js_count < n_frames || rgb_count < n_frames || ir_count < n_frames) {
      ros::spinOnce();
    }
    // afterwards finish program
    cout << "\ncollected " << js_count << " frames" << endl;
    cout << "Done saving images and joint states." << endl;
    return 0;
  }

  if (receive_frames == true) {
    boost::filesystem::remove_all(path + "/imgs/rgb");
    boost::filesystem::remove_all(path + "/imgs/ir");
    boost::filesystem::create_directory(path + "/imgs");
    boost::filesystem::create_directory(path + "/imgs/rgb");
    boost::filesystem::create_directory(path + "/imgs/ir");
    image_transport::Subscriber rgb_sub = it.subscribe("/calibration_rgb_img", 1, rgbImageWrite);
    image_transport::Subscriber ir_sub = it.subscribe("/calibration_ir_img", 1, irImageWrite);
    ros::Subscriber joint_states_sub = nh.subscribe("/calibration_joint_states", 1, jointStatesWrite);
    std::cout << "\nready to receive frames" << endl;
    while (js_count < n_frames || rgb_count < n_frames || ir_count < n_frames) {
      ros::spinOnce();
    }
    cout << "\ncollected " << js_count << " frames" << endl;
    cout << "starting camera calibration" << endl;
    cameraCalibration();
  }
  // setting to read provided files
  if (from_folder == true) {
    readJointStates();
    readCheckPoses();
  }
  // setting to read generated files
  if (from_folder_ == true) {
    cout << "starting camera calibration" << endl;
    readJoint_States();
    cameraCalibration();
  }

  if (QR24 == true) {
    handEyeQR24();
    handEyeError();
    writeResults("QR24");
  }

  if (standardHandEye == true) {
    handEye();
    handEyeError();
    writeResults("Tsai-Lenz");
  }
  // service to advertise hand eye results
  //ros::ServiceServer service = nh.advertiseService("handEyeCalibration", handEyeOutput);

  return 0;
}