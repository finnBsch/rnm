//
// Created by lars on 07.06.21.
//
#include "ros/ros.h"
#include <iostream>
#include <stdlib.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <stdio.h>

using namespace std;
using namespace cv;

//Perform the stereoCalibration
//TODO Die folgenden Variablen müssen sowohl von dem loadFromFolder als auch von dem loadRealTime hierhin übergeben werden.
//irFrameSize;
//rgbFrameSize;
//vector<vector<Point3f> > object_points;
//vector<vector<Point2f> > imagePoints1, imagePoints2;
//vector<vector<Point2f> > left_img_points, right_img_points;
//vector<vector<Point2f> > rgbcorners;
//vector<vector<Point2f> > ircorners;
//checkerBoard
//fieldSize




int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_calibration");
  ros::NodeHandle n;


  std::vector<std::vector<cv::Point3f>> irObjP;   // Checkerboard world coordinates
  std::vector<std::vector<cv::Point3f>> rgbObjP;  // Checkerboard world coordinates

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



  // Calibrate the rgb frame intrinsics
  cv::Mat rgbK;  // calibration Matrix K
  Mat rgbk = (Mat1d(1, 8));
  std::vector<cv::Mat> rvecs, tvecs;
  std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
  int flags = cv::CALIB_FIX_S1_S2_S3_S4 + cv::CALIB_FIX_TAUX_TAUY +
              cv::CALIB_RATIONAL_MODEL;  // used to compute k4-k6

  std::cout << "Calibrating rgb intrinsics...\n" << std::endl;

  float rgberror =
      cv::calibrateCamera(rgbObjP, rgbcorners, rgbFrameSize, rgbK, rgbk, rvecs, tvecs, flags);
  std::cout << "Reprojection error of rgb frames = " << rgberror << "\nK =\n"
            << rgbK << "\nk=\n"
            << rgbk << std::endl;
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

  ///////////////////////////////////////////////////////
  vector<Point3f> obj;
  for (int j = 1; j < checkerBoard[1]; j++) {
    for (int k = 1; k < checkerBoard[0]; k++) {
      obj.push_back(Point3f((float)k * fieldSize, (float)j * fieldSize, 0));
    }
  }

  std::cout << "Starting Calibration\n";
  cv::Mat K1, K2, R, F, E;
  cv::Vec3d T;
  cv::Mat D1, D2;
  K1 = rgbK;
  K2 = irK;
  D1 = rgbk;
  D2 = irk;

  int stereoflags =
      CALIB_FIX_INTRINSIC + CALIB_FIX_S1_S2_S3_S4 + CALIB_FIX_TAUX_TAUY + CALIB_RATIONAL_MODEL;

  std::cout << "Compute stereocalibration...\n";
  double rms = stereoCalibrate(object_points, left_img_points, right_img_points, K1, D1, K2, D2,
                               rgbFrameSize, R, T, E, F, stereoflags);
  std::cout << "Done with RMS error=" << rms << endl;

  // Precompute lens correction interpolation
  std::cout << "Done Stereocalibration\n";

  // Rectification
  std::cout << "Starting Rectification\n";
  cv::Mat R1, R2, P1, P2, Q;
  stereoRectify(K1, D1, K2, D2, irFrameSize, R, T, R1, R2, P1, P2, Q);
  printf("Done Rectification\n");

  // Save as YML
  cv::FileStorage fs1("CalibrationParam.yml", cv::FileStorage::WRITE);
  fs1 << "K1" << K1;
  fs1 << "K2" << K2;
  fs1 << "D1" << D1;
  fs1 << "D2" << D2;
  fs1 << "R" << R;
  fs1 << "T" << T;
  fs1 << "E" << E;
  fs1 << "F" << F;
  fs1 << "R1" << R1;
  fs1 << "R2" << R2;
  fs1 << "P1" << P1;
  fs1 << "P2" << P2;
  fs1 << "Q" << Q;

  // Show lens corrected ir images
  //  for (auto const &f : irFileNames) {
  //     std::cout << std::string(f) << std::endl;
  //     cv::Mat irimg = cv::imread(f, cv::IMREAD_COLOR);
  //     cv::Mat irimgUndistorted;
  // 5. Remap the image using the precomputed interpolation maps.
  //      cv::remap(irimg, irimgUndistorted, irmapX, irmapY, cv::INTER_LINEAR);
  // Display the undistorted images
  //  cv::imshow("undistorted image", irimgUndistorted);
  //  cv::waitKey(0);
  //   }
//TODO Die Rückgabe an den Cameradriver fehlt. Herausfinden wie man ihm das .yml file übergeben muss.


  cout << "Calibration successful" << std::endl;
  return 0;
}