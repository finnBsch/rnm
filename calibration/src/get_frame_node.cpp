#include <iostream>
#include <string>
#include <sstream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

// if run, program waits until any key is pressed and then saves rgb, ir, depth images
// to path + /calibration/cal_imgs/

// path to folder containing the package (calibration)
string path = "/home/nico/CLionProjects/rnm_ws/src";

// counters for the filenames
int pose_rgb = 1;
int pose_ir = 1;
int pose_depth = 1;

////////////////////////////////////////////////////////////////////////////////////////////////////

// conversion of depth image
void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img) {
  //Process images
  if (mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols) {
    mono8_img = cv::Mat(float_img.size(), CV_8UC1);
  }
  cv::convertScaleAbs(float_img, mono8_img, 100, 0.0);
}

void depthCallback(const sensor_msgs::ImageConstPtr& original_image) {
  cv_bridge::CvImagePtr cv_ptr;
  //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
  try {
    //Always copy, returning a mutable CvImage
    //OpenCV expects color images to use BGR channel order.
    cv_ptr = cv_bridge::toCvCopy(original_image);
  }
  catch (cv_bridge::Exception &e) {
    //if there is an error during conversion, display it
    ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
    return;
  }

  // now converting from 16bit (depth img) to 8bit, loss is small
  cv::Mat depth_float_img = cv_ptr->image;
  cv::Mat depth_mono8_img;
  depthToCV8UC1(depth_float_img, depth_mono8_img);
  cv::imshow("frame", depth_mono8_img);
  cv::waitKey(1);
}


// writing depth image
void depthImageWrite(const sensor_msgs::ImageConstPtr& msg) {
  try {
    // converting from 16 bit to 8 bit :/ ... 5 bits are apparently not used for image data
    // thus only 11 to 8 conversion ... still lossy
    cv::Mat depth_float_img = cv_bridge::toCvCopy(msg)->image;
    cv::Mat img;
    depthToCV8UC1(depth_float_img, img);

    // create filename
    stringstream ss;
    string name = "/calibration/cal_imgs/depth/pose_";
    string type = ".jpg";
    ss << path << name << (pose_depth) << type; // pose = counter, set at start
    string filename = ss.str();
    ss.str("");
    // write img
    imwrite(filename, img);
    std::cout << "depth: Pose_" << pose_depth << " img written." << endl;
    pose_depth++;
  }
  catch (cv_bridge::Exception &e) {
    std::cout << "Could not write depth: Pose_" << pose_depth-1 << " image." << endl;
  }
}


// ir image writing
void irImageWrite(const sensor_msgs::ImageConstPtr& msg) {
  try {
    // convert image from ros msg to bgr8 (cv)
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

    // create filename
    stringstream ss;
    string name = "/calibration/cal_imgs/ir/pose_";
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
    string name = "/calibration/cal_imgs/rgb/pose_";
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

int main(int argc, char** argv) {
  ros::init(argc, argv,"frame_reader");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // callbacks. each image type has its own saving callback
  image_transport::Subscriber rgb_sub = it.subscribe("/k4a/rgb/image_raw", 1, rgbImageWrite);
  image_transport::Subscriber ir_sub = it.subscribe("/k4a/ir/image_raw", 1, irImageWrite);
  image_transport::Subscriber depth_sub = it.subscribe("/k4a/depth/image_raw", 1, depthImageWrite);

  // wait until any key is pressed
  cout << "\n\nPress any key to save images.\n";
  while (cin.ignore()) {
    ros::spinOnce(); //call callbacks
  }
  //ros::spin();
  return 0;
}