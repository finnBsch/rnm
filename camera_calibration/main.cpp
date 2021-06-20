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

int pose = 1;// just a counter for the filenames

// frames to extract
const int n_poses = 44; // number of different frames to extract
int time_stamps[n_poses] = {1616052220, 1616052227, 1616052235, 1616052244, 1616052256,
                            1616052267, 1616052276, 1616052284, 1616052293, 1616052303,
                            1616052313, 1616052325, 1616052332, 1616052341, 1616052352,
                            1616052363, 1616052373, 1616052379, 1616052387, 1616052396,
                            1616052406, 1616052412, 1616052423, 1616052435, 1616052442,
                            1616052454, 1616052465, 1616052477, 1616052489, 1616052498,
                            1616052507, 1616052518, 1616052528, 1616052538, 1616052546,
                            1616052555, 1616052566, 1616052575, 1616052583, 1616052592,
                            1616052603, 1616052611, 1616052621, 1616052629};
// list for already saved imgs, to avoid saving multiple images of same time stamp
int time_stamps_saved[n_poses] = {};

// function for showing the images from sensor_imgs; default: not used
void imageCallback(const sensor_msgs::ImageConstPtr& msg){
//    const ros::Time time_stamp = msg->header.stamp;

// converting from ros to cv (mat) format, pointer to image from cv_bridge::CvImage
  try{
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::imshow("frame", img);
    cv::waitKey(100);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  //ROS_INFO("frame_id: %s ; time: %s", frame_id, time);
  //ROS_INFO("height: %i , width: %i", height, width);
  ROS_INFO("seq: %i", msg->header.seq); // starting with 732
  ROS_INFO("time stamp: %d", msg->header.stamp); // starting with 1616052219
  ROS_INFO("frame_id: %s\n", msg->header.frame_id.c_str()); // ?
}

// function for the image writing process
void imageWrite(const sensor_msgs::ImageConstPtr& msg) {
  try {
    // convert image from ros msg to bgr8 (cv)
    cv::Mat imgw = cv_bridge::toCvShare(msg, "bgr8")->image;
    // const ros::Time time(time_stamps[i]); // creates ros time stamp from int
    int stamp = msg->header.stamp.toSec(); // cut off decimals of seconds and convert to int
    // loop through all poses
    for (int i = 0; i < n_poses; i++) {
      if (stamp == time_stamps[i]) { // if time stamp == any of pre-entered stamp (there seems to be a faster function for this (std::any_of))
        // check whether already saved an image with the same time stamp
        if (time_stamps[i] == time_stamps_saved[i]) {} // do nothign if already saved
        else {
          // create filename
          stringstream ss;
          string name = "/home/nico/catkin_ws/src/frame_reader/cal_imgs/pose_";
          string type = ".jpg";
          ss << name << (pose) << type; // pose = counter, set at start
          string filename = ss.str();
          ss.str("");
          // write img
          imwrite(filename, imgw);
          // put time stamp into list of already saved imgs to avoid doublets
          time_stamps_saved[i] = time_stamps[i];
          std::cout << "Pose_" << pose << " img written." << endl;
          pose++;
        }
      }
    }
  }
  catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str()); //could also be anything of the above
  }
  //ROS_INFO("frame_id: %s ; time: %s", frame_id, time);
  //ROS_INFO("height: %i , width: %i", msg->height, msg->width);
  //ROS_INFO("seq: %i", msg->header.seq); // starting with 732
  ROS_INFO("time stamp: %d", msg->header.stamp); // starting with 1616052219
  //ROS_INFO("frame_id: %s\n", msg->header.frame_id.c_str()); //
}

int main(int argc, char** argv) {
  ros::init(argc, argv,"frame_reader");
  ros::NodeHandle nh;

  // uncomment to: read only
  //cv::namedWindow("frame");
  //ros::Subscriber sub = nh.subscribe("k4a/rgb/image_raw", 1, imageCallback); // apparently transport should always be used for imgs

  // uncomment to: write imgs
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("k4a/rgb/image_raw", 1, imageWrite);

  // subscribing at frequency
  // while loop needed
  ros::Rate rate(4 /*Hz*/);
  while (ros::ok()) {
    ros::spinOnce(); // use this when using rosrate
    rate.sleep();
  }
  //ros::spin(); // use this without rosrate, loops automatically
  cv::destroyWindow("frame");
  return 0;
}