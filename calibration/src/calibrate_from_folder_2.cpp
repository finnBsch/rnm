//
// Created by lars on 02.06.21.
//
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
//#include "calibration/imagepair.h"
#include "iostream"
#include <tuple>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;


void LoadFromFolder(){
  cv::Size rgbFrameSize(2048, 1536);
  cv::Size irFrameSize(640, 576);
  std::vector<cv::String> rgbFileNames;
  std::vector<cv::String> irFileNames;
  std::string rgbFolder("/home/lars/CLionProjects/CameraCalibrationtests/cal_imgs/rgb/*.jpg");
  cv::glob(rgbFolder, rgbFileNames, true);  // load rgb images into opencv
  std::string irFolder("/home/lars/CLionProjects/CameraCalibrationtests/cal_imgs/ir/*.jpg");
  cv::glob(irFolder, irFileNames, true);

  std::vector<std::vector<cv::Point3f>> irObjP;   // Checkerboard world coordinates
  std::vector<std::vector<cv::Point3f>> rgbObjP;  // Checkerboard world coordinates

  // Define checkerboard Parameters for both frames
  int fieldSize = 40;
  int checkerBoard[2] = {9, 6};        // Checkerboard pattern
  cv::Size patternSize(9 - 1, 6 - 1);  // Number of inner corners per a chessboard row and column


  std::vector<std::vector<cv::Point2f>> rgbcorners(rgbFileNames.size());  // corners in rgb frames
  std::vector<std::vector<cv::Point2f>> ircorners(irFileNames.size());    // corners in ir frames
  std::vector<cv::Point2f> rgbimgp;  // Define the rgb img point
  std::vector<cv::Point2f> irimgp;   // Define the ir img point

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
  std::size_t i = 0;
  for (auto const& f : rgbFileNames) {
    std::cout << std::string(f) << std::endl;
    rgbimg = cv::imread(rgbFileNames[i]);  // Load the images
    cv::Mat rgbgray;                       // grayscale the image
    cv::cvtColor(rgbimg, rgbgray, cv::COLOR_RGB2GRAY);
    bool rgbPatternFound = cv::findChessboardCorners(
        rgbgray, patternSize, rgbcorners[i],
        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FILTER_QUADS);
    // Use cv::cornerSubPix() to refine the found corner detections with default values given by opencv
    if (rgbPatternFound) {
      cv::cornerSubPix(
          rgbgray, rgbcorners[i], cv::Size(11, 11), cv::Size(-1, -1),
          cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.00001));
      rgbObjP.push_back(rgbobjp);
    } else {
      rgbcorners.erase(next(rgbcorners.begin(), i));
      cout << endl << "!!!!!!!!File " << rgbFileNames[i] << "not used!!!!!!!!" << endl << endl;
    }

    // Display the detected pattern on the chessboard
    // cv::drawChessboardCorners(rgbimg, patternSize, rgbcorners[i], rgbPatternFound);
    // cv::imshow("RGB chessboard corner detection", rgbimg);
    // cv::waitKey(1);
    i++;
  }
  cout << "All RGB Corners detected and safed in rgbcorners\n";



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



  //Cornerdetection for the Stereocalibration
  vector<vector<Point3f> > object_points;
  vector<vector<Point2f> > imagePoints1, imagePoints2;
  vector<Point2f> corners1, corners2;
  vector<vector<Point2f> > left_img_points, right_img_points;
  Mat img1, img2;
  //Size board_size = Size(board_width, board_height);
  // int board_n = board_width * board_height;
  int num_deleted = 0;
  for (int i = 0; i < rgbFileNames.size(); i++) {


    img1 = cv::imread(rgbFileNames[i]); //Load the images
    img2 = cv::imread(irFileNames[i]); //Load the images
    cv::Mat gray1;//grayscale the rgb image
    cv::cvtColor(img1, gray1, cv::COLOR_RGB2GRAY);
    cv::Mat gray2;//grayscale the ir image
    cv::cvtColor(img2, gray2, cv::COLOR_RGB2GRAY);

    bool found1 = cv::findChessboardCorners(gray1, patternSize, corners1,
                                            CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_FILTER_QUADS);

    bool found2 = cv::findChessboardCorners(gray2, patternSize, corners2,
                                            CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);

    std::vector<cv::Point3f> obj;
    for (int j = 1; j < checkerBoard[1]; j++){
      for (int k = 1; k < checkerBoard[0]; k++) {
        obj.push_back(Point3f((float) k * fieldSize, (float) j * fieldSize, 0));
      }
    }
    if (found1 && found2) {
      //Refine CornerDetection
      cv::cornerSubPix(gray1, corners1, cv::Size(20, 20), cv::Size(-1, -1),
                       cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.1));

      cv::drawChessboardCorners(gray1, patternSize, corners1, found1);

      cv::cornerSubPix(gray2, corners2, cv::Size(20, 20), cv::Size(-1, -1),
                       cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.1));

      cv::drawChessboardCorners(gray2, patternSize, corners2, found2);
      cout << i << "good framepairs" << endl;
      imagePoints1.push_back(corners1);
      imagePoints2.push_back(corners2);
      object_points.push_back(obj);
      cout << "leftImg: " << rgbFileNames[i] << " and rightImg: " << irFileNames[i] << endl;
    }
    else {
      corners1.erase(next(corners1.begin(), i-num_deleted));
      corners2.erase(next(corners2.begin(), i-num_deleted));
      num_deleted++;

      cout << endl << "!!!!!!!!Pairs " << irFileNames[i] << "not used!!!!!!!!" << endl << endl;
    }
  }

  for (int i = 0; i < imagePoints1.size(); i++) {
    vector<Point2f> v1, v2;
    for (int j = 0; j < imagePoints1.size(); j++) {
      v1.push_back(Point2f((double) imagePoints1[i][j].x, (double) imagePoints1[i][j].y));
      v2.push_back(Point2f((double) imagePoints2[i][j].x, (double) imagePoints2[i][j].y));
    }
    left_img_points.push_back(imagePoints1[i]);
    right_img_points.push_back(imagePoints2[i]);
  }

  cout << "all Corners found for the stereocalibration" << endl;
}


int main(int argc, char** argv)  {
  ros::init(argc, argv, "loadFromFolder_no");

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
  //ros::ServiceServer service = nh.advertiseService("get_endeffector", get_end_effector);
  ros::spin();
}