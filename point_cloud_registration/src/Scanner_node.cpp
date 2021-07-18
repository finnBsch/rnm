#include <eigen_conversions/eigen_msg.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "point_cloud_registration/PCJScombined.h"

class Scanner {
 public:
  Scanner(ros::NodeHandle& nh)
      : nh_(nh),
        nextPose(0),
        pub_(nh_.advertise<point_cloud_registration::PCJScombined>("PCJScombined", 1)),
        pose_publisher_(nh_.advertise<std_msgs::Float64MultiArray>("/move_command",10))
  {
    for (int i = 0; i < 14; ++i) {
      goal_poses[i].resize(7);
    }
    goal_poses[0] << 0.29956781262726967, -0.8363118570568921, -0.355985664347674, -2.404824108907839, -1.2963598661687292, 3.1518151527810203, -1.3188153382455798;
    goal_poses[1] << 0.4259447902980896, -0.9795811651255881, 0.04120909847846797, -2.5550065172013654, -1.775420829713692, 2.8372738042791625, -0.7576419231928546;
    goal_poses[2] << 0.4259889913978997, -0.2827405736410619, -0.20020773148216975, -2.095111846337074, -1.6355848177597845, 2.9976366702042396, -0.8040525674656157;
    goal_poses[3] << 0.16119604241799315, 0.030798848581041975, 0.02847667100143907, -1.8977313320284104, -1.157083429473139, 3.1101174058244205, -1.3022494153455342;
    goal_poses[4] << 0.10909638623553938, 0.10933573455296024, 0.0470835746169001, -2.017292373554503, -1.0792524768171254, 3.031505150760735, -1.3065926993053536;
    goal_poses[5] << 0.5581884180361546, -1.279744577499401, 0.7845640344246225, -2.6862266177586758, -1.3883913449917973, 2.2693456432024637, -0.7940945269958923;
    goal_poses[6] << -0.24607207588732755, -0.8347868611963895, 0.6109545837433874, -2.2991939405809814, -0.9207846552403848, 2.903616721312206, -1.423861813741188;
    goal_poses[7] << 0.31608961861092894, -1.0025074334264752, -0.47237351381543485, -2.5225443878199516, -1.034371766584036, 3.102049557008643, -1.4242259458041098;
    goal_poses[8] << 1.046786975870738, -0.7351043986623597, -1.0846537904013245, -2.1289558577313774, -1.2424869881887204, 3.0340201482335747, -1.5428989303143705;
    goal_poses[9] << 0.3004847149182361, -1.4650641730507132, -0.5447432433731129, -2.635153668908044, -1.227644657438089, 3.0535759518993646, -1.5333310116184855;
    goal_poses[10] << 0.19644559295141714, -1.7341250871691614, -0.17035317445101836, -2.8411875708814383, -1.1426190268027516, 3.1322373140849535, -1.534727002856544;
    goal_poses[11] << -0.16235475534173158, -1.7496607414199081, 0.4075235850688135, -2.781168749448149, -1.0431851111335788, 3.247316841468949, -0.9767711835301054;
    goal_poses[12] << 0.3127231318565338, -1.3920047436517309, -1.0239938684037722, -2.5035714420264643, -1.6110405030110022, 3.038243901546581, -1.1388881275143914;
    goal_poses[13] << -1.0734086892939454, -0.637630518185465, 1.443866974093667, -1.8275403487095305, -0.8237624124027089, 2.998936798472645, -1.5012728818551468;
  }

 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Publisher pose_publisher_;
  ros::Subscriber sub_;
  std::array<Eigen::VectorXd, 14> goal_poses;

  int nextPose;
  const int js_queue_size = 500;
  const int rounding_precision = 10000;

  //function from which we will send our goal pose to the path planning node
  void sendGoalPose(){
    if (nextPose == 0){
      ros::Duration(1).sleep();
    }
    std_msgs::Float64MultiArray outputPose;
    tf::matrixEigenToMsg(goal_poses[nextPose], outputPose);
    pose_publisher_.publish(outputPose);
  }

  //subscribe to to the joint_states topic
  void getPose(){
    std::cerr << "started get pose" << std::endl;
    sub_ = nh_.subscribe<sensor_msgs::JointState>("/franka_state_controller/joint_states_desired", js_queue_size, boost::bind(&Scanner::comparePose, this, _1));
    ros::spin();
  }

  //check if the joint states reached the desired position. if so, increment the nextPose counter and enter the getPointCloud function
  void comparePose(const sensor_msgs::JointStateConstPtr& joint_state){
    sensor_msgs::JointState tempJS;
    sensor_msgs::JointState rounded_goal_pose;

    for (int i = 0; i < 7; ++i) {
      tempJS.position.push_back(round(joint_state->position[i]*rounding_precision)/rounding_precision);
      rounded_goal_pose.position.push_back(round(goal_poses[nextPose][i]*rounding_precision)/rounding_precision);
    }

    if (rounded_goal_pose.position == tempJS.position){
      std::cerr << "goal pose reached!" << std::endl;
      ros::Duration(0.2).sleep();
      nextPose++;
      getPointCloud();
    }
  }

  //Subscribe to the pointcloud and jointstates topic and match them by time stamp
  //boolean "published_message", to check (in the callback function) if there was already an message published for this goal pose
  void getPointCloud(){

    PC = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/k4a/points2");
    sensor_msgs::JointStateConstPtr JS = ros::topic::waitForMessage<sensor_msgs::JointState>("/franka_state_controller/joint_states_desired"); //franka_state_controller/joint_states_desired

       point_cloud_registration::PCJScombined PCJS;
       PCJS.PC = *PC;
       PCJS.JS = *JS;
       PCJS.PC.header.frame_id = "rgb_camera_link";
       pub_.publish(PCJS);
       if (nextPose<sizeof(goal_poses)) {
         sendGoalPose();
       }

  }


  //public function to start the scanning, first send a goal pose and then start checking, if it has been reached yet
 public:
  void startScan(){
      sendGoalPose();
      getPose();
  }
};

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "Scanner_node");
  ros::NodeHandle nh("~");

  Scanner scan(nh);
  scan.startScan();
}

