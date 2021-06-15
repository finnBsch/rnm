#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include "point_cloud_registration/PCJScombined.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

class Scanner {
 public:
  Scanner(ros::NodeHandle& nh)
      : nh_(nh),
        nextPose(0),
        pub_(nh_.advertise<point_cloud_registration::PCJScombined>("PCJScombined", 1))
  {
    //example goal poses, taken from the rosbag
    //TODO: decide on poses we want to take images from (joint or cartesian space?)
    goal_poses[0].position = {-0.12284345045716151, 0.2267075755956995, 0.39165609702425463, -1.6740154526567326, -0.8181892938154488, 2.8873320053625218, -2.394065079622183};
    goal_poses[1].position = {0.21486264859869972, -1.0896853024838538, -0.988342189261048, -2.392936109010114, -1.8435856154785526, 2.934521175608565, -1.4491195717718865};
  }
 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  std::array<sensor_msgs::JointState, 2> goal_poses;
  message_filters::Subscriber<sensor_msgs::PointCloud2> filtered_cloud;
  message_filters::Subscriber<sensor_msgs::JointState> jointStates;
  bool published_message;
  int nextPose;
  const int pc_queue_size = 50;
  const int js_queue_size = 500;

  //function from which we will send our goal pose to the path planning node
  void sendGoalPose(){

  }

  //subscribe to to the joint_states topic
  void getPose(){
    std::cerr << "started get pose" << std::endl;
    sub_ = nh_.subscribe<sensor_msgs::JointState>("joint_states", js_queue_size, boost::bind(&Scanner::comparePose, this, _1));
    ros::spin();
  }

  //check if the joint states reached the desired position. if so, increment the nextPose counter and enter the getPointCloud function
  //TODO: implement an approximate comparison, since the actual joint states will not exactly become the desired ones
  void comparePose(const sensor_msgs::JointStateConstPtr& joint_state){
    if (goal_poses[nextPose].position == joint_state->position){
      std::cerr << "goal pose reached!" << std::endl;
      nextPose++;
      getPointCloud();
    }
  }

  //Subscribe to the pointcloud and jointstates topic and match them by time stamp
  //boolean "published_message", to check (in the callback function) if there was already an message published for this goal pose
  void getPointCloud(){
    std:cerr << "entered getPC" << std::endl;
    published_message = false;
    filtered_cloud.subscribe(nh_, "/points2", pc_queue_size);
    jointStates.subscribe(nh_, "/joint_states", js_queue_size);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::JointState> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(js_queue_size), filtered_cloud, jointStates);
    sync.registerCallback(boost::bind(&Scanner::publishMessage, this, _1, _2));
    ros::spin();


    //when we work with static poses, there is no need for time stamp matching, so we could just take the next incoming pointcloud and jointstates and publish them as a combined message
    //this would also make the callback function (publishMessage) unnecessary
    /*    sensor_msgs::PointCloud2ConstPtr PC = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("points2");
    sensor_msgs::JointStateConstPtr JS = ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
    point_cloud_registration::PCJScombined PCJS;
    PCJS.PC = *PC;
    PCJS.JS = *JS;
    pub_.publish(PCJS);
    sendGoalPose();*/
  }

  //callback function to publish the matched pc and js
  //first check, if there was already a message published for this pose. if so, do nothing
  //otherwise, unsubscribe from the topics, so no more messages come in and publish the received pc+js
  //as a last step, send the next goalPose
  void publishMessage(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const sensor_msgs::JointStateConstPtr& joint_states){
    std:cerr << "entered publish message" << std::endl;
      if(published_message){
        return;
      }
    filtered_cloud.unsubscribe();
    jointStates.unsubscribe();
    point_cloud_registration::PCJScombined PCJS;
    PCJS.PC = *cloud_msg;
    PCJS.JS = *joint_states;
    pub_.publish(PCJS);
    published_message = true;
    if (nextPose<sizeof(goal_poses)){
      sendGoalPose();
    }else{
      nh_.shutdown();
      //this does not shut down the whole node for some reason
      //TODO: find a better solution
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
  ros::init (argc, argv, "PCStitch");
  ros::NodeHandle nh;

  Scanner scan(nh);
  scan.startScan();

  ros::spin ();
}

