//
// Created by lars on 02.06.21.
//
#ifndef SRC_IMAGEPROCESSOR_H
#define SRC_IMAGEPROCESSOR_H
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include "calibration/imagepair.h"
#include "iostream"
#include <tuple>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>


using namespace std;
using namespace cv;

class imageprocessor {
 private:
  //Subscriber
  ros::NodeHandle n;
  image_transport::Subscriber sub_rgb;
  image_transport::Subscriber sub_ir;
  vector<tuple< int, cv::Mat, cv::Point2f>> collected_frame;
  vector<cv::Mat> collected_irs;
  image_transport::ImageTransport* it;

  //Declaration of constants
  const int imgsum = 30; //images number needed for the calibration
  const int w = 9;
  const int h = 6;
  const int fieldsize = 40; // fieldsize in mm
  const int checkerBoard[2] = {w, h};
  const Size patternSize = Size(w-1, h-1); // Number of inner corners per a chessboard row and column



void loadFromFolder();



void test_ir(const sensor_msgs::ImageConstPtr& msg);

 public:
  imageprocessor(string rgbTopic, string irTopic);
};
#endif
