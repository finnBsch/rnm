//
// Created by lars on 21.06.21.
//

#ifndef SRC_CV_HANDEYE_H
#define SRC_CV_HANDEYE_H
#include <ros/ros.h>
//#include <tf2_msgs/TFMessage.msg>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <opencv/cv.hpp>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

//#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
using namespace cv;
Mat homogeneousInverse(const Mat& T);
//void calibrateHandEyeQR24(std::vector<Mat>& Hg, const std::vector<Mat>& Hc, Mat& R_cam2gripper, Mat& t_cam2gripper);
void calibrateHandEye_(InputArrayOfArrays R_gripper2base, InputArrayOfArrays t_gripper2base, InputArrayOfArrays R_target2cam, InputArrayOfArrays t_target2cam, OutputArray R_cam2gripper, OutputArray t_cam2gripper);
void calibrateHandEyeTsai_(const std::vector<Mat>& Hg, const std::vector<Mat>& Hc, Mat& R_cam2gripper, Mat& t_cam2gripper);
Mat rot2quatMinimal(const Mat& R);
Mat rot2quat(const Mat& R);
Mat skew(const Mat& v);
Mat quatMinimal2rot(const Mat& q);
#endif  // SRC_CV_HANDEYE_H
