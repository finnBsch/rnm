#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include "point_cloud_registration/PCJScombined.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

ros::Publisher publisher;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const sensor_msgs::JointStateConstPtr& joint_states, int& msg_counter)
{
  if (msg_counter % 10 == 0){
    point_cloud_registration::PCJScombined PCJS;
    PCJS.PC = *cloud_msg;
    PCJS.JS = *joint_states;
    publisher.publish(PCJS);
    msg_counter++;
  } else {
    msg_counter++;
  }
}

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "Synchronizer_node");
  ros::NodeHandle nh("~");
  int msg_count = 0;
  //Let the publisher publish on topic transformed_clouds
  publisher = nh.advertise<point_cloud_registration::PCJScombined> ("PCJScombined", 50);

  //Subscribe to the PC and JointState topic, large queue size where all messages from the bag can fit in
  //-> adjust queue size later
  message_filters::Subscriber<sensor_msgs::PointCloud2> filtered_cloud(nh, "/points2", 50);
  message_filters::Subscriber<sensor_msgs::JointState> joint_states(nh, "/joint_states", 500);

  //Initialize approximate time filter, to fit together PCs and joint states by time stamp
  //again a large queue size to make sure no messages get lost
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::JointState> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(500), filtered_cloud, joint_states);
  sync.registerCallback(boost::bind(&cloud_cb, _1, _2, msg_count));
  ros::spin ();
}
