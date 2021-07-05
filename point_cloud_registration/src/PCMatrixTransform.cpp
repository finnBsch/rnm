#include <iostream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "forward_kin/get_endeffector.h"
#include "point_cloud_registration/PCJScombined.h"

//Public publisher
ros::Publisher publisher;

//for testing only: counter to see how many times the callback function has been called
int msg_counter = 0;

using namespace Eigen;

//global cloud to store the stitched cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr stitched_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

VectorXd currentA_vector(12, 1);


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const sensor_msgs::JointStateConstPtr& joint_states,
              ros::NodeHandle &nh)
//void cloud_cb (const point_cloud_registration::PCJScombined::ConstPtr& PCJS, ros::NodeHandle &nh)
{
  //print out how many messages have been received in total
  msg_counter++;
  std::cerr <<" \nReceived messages: ";
  std::cerr << msg_counter;
  std::cerr <<"\n";

  //only take every 20th message for stitching
  if (msg_counter%20 == 0){


  //SUBSAMPLE---------------------------------------------------------------------------------------------------------------
  //Initialize container for PC message, for the filtered cloud and pointer to PC message
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 pc2_cloud_subsample;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);
  //pcl_conversions::toPCL(PCJS->PC, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
  vg.setInputCloud (cloudPtr);
  vg.setLeafSize (0.003, 0.003, 0.003);
  vg.filter (pc2_cloud_subsample);

  //REMOVING OUTLIERS---------------------------------------------------------------------------------------
  //Initialize container for input and output cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZRGB> ());
 // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB> ());

  //write filtered cloud in input cloud
  pcl::fromPCLPointCloud2(pc2_cloud_subsample, *cloud_xyz);


    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud_xyz << std::endl;

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud_xyz);
    sor.setMeanK(50);
    sor.setStddevMulThresh(0.5);
    sor.filter(*cloud_xyz);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_xyz << std::endl;


  //TRANSFORMATION---------------------------------------------------------------------------------------------------------
  //Initialize container for output cloud
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB> ());

  //hand-eye calibration matrix
  Matrix4f handeye;
  handeye << -0.0342177, -0.0224303,   0.999141,  0.0357424,
          -0.998979, -0.0285611,  -0.034854, -0.0232153,
          0.0293307,  -0.999324, -0.0214294,  0.0547717,
          0,          0,          0,          1;



    forward_kin::get_endeffector srv;
   // sensor_msgs::JointState joint_state_msg;
    //joint_state_msg  = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",ros::Duration(10)));

    ros::ServiceClient client = nh.serviceClient<forward_kin::get_endeffector>("forward_kin_node/get_endeffector");
    boost::array<double, 7> angles = {joint_states->position[0], joint_states->position[1],joint_states->position[2],
                                   joint_states->position[3], joint_states->position[4],joint_states->position[5],
                                   joint_states->position[6]};
   /* boost::array<double, 7> angles = {PCJS->JS.position[0], PCJS->JS.position[1],PCJS->JS.position[2],
                                      PCJS->JS.position[3], PCJS->JS.position[4],PCJS->JS.position[5],
                                      PCJS->JS.position[6]};*/

    srv.request.joint_angles = angles;
// --------------- GET A MATRIX -------------------
    // check connection
    auto cl = client.call(srv);
    if (cl)
    {
      ROS_INFO("forward_kin service called");
    }
    else
    {
      ROS_ERROR("Failed to call service forward_kin");
      exit; //TODO find better solution
    }
    // get the current Transformation matrix Matrix from the forward_kin node
    Matrix4f transform;
    transform << srv.response.layout.data[0],srv.response.layout.data[1],srv.response.layout.data[2],srv.response.layout.data[3],
        srv.response.layout.data[4],srv.response.layout.data[5],srv.response.layout.data[6],srv.response.layout.data[7],
        srv.response.layout.data[8],srv.response.layout.data[9],srv.response.layout.data[10],srv.response.layout.data[11],
        srv.response.layout.data[12],srv.response.layout.data[13],srv.response.layout.data[14],srv.response.layout.data[15];
   transform = transform*handeye;

  //Transformation of the filtered cloud
  pcl::transformPointCloud (*cloud_xyz, *cloud_xyz, transform);


  //CROPPING-------------------------------------------------------------------------------------------------------------------------
  //Crop the point cloud, so it mainly contains the skeleton
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cropped (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::CropBox<pcl::PointXYZRGB> crop;
  crop.setMin(Vector4f(0.15, -0.5, 0.02, 1.0));
  crop.setMax(Vector4f(0.7, 0.5, 0.3, 1.0));
  crop.setInputCloud(cloud_xyz);
  crop.filter(*cloud_xyz);

  //ICP--------------------------------------------------------------------------------------------------------------------------------
  //using ICP for fine alignment

  //If condition to test if the total stitched cloud is still empty. If so fill it with the arrived cloud.
  // Else stitch the arrived cloud to the total stitched cloud
  if (stitched_cloud->empty() == 1){
    std::cout << "first initialization";
    *stitched_cloud = *cloud_xyz;
  }else{
    //Set clouds for the ICP algorithm
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr goal_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource(cloud_xyz);
    icp.setInputTarget(stitched_cloud);

    //ICP parameters (examples), we still need to adjust them
    //icp.setEuclideanFitnessEpsilon(1);
    //icp.setTransformationEpsilon(1e-9);
    icp.setMaximumIterations(10);

    //align new cloud to stitched cloud and add to the total stitched cloud.
    icp.align(*goal_cloud);
    *stitched_cloud += *goal_cloud;
    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  }

  //Define a PC for output and publish it
  sensor_msgs::PointCloud2 output;

  pcl::toROSMsg(*stitched_cloud, output);
  publisher.publish (output);
  // Safe stitched cloud from bagfile
  if( msg_counter == 140){
    pcl::io::savePCDFile( "/home/niklas/Documents/RNM/stitched_cloud.pcd", *stitched_cloud, true ); // Binary format
  }
  }
}

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "PCMatrixTransform");
  ros::NodeHandle nh;
  //Let the publisher publish on topic transformed_clouds
  publisher = nh.advertise<sensor_msgs::PointCloud2> ("transformed_clouds", 1);

//  ros::Subscriber sub = nh.subscribe<point_cloud_registration::PCJScombined>("/PCJScombined", 50, boost::bind(&cloud_cb, _1, boost::ref(nh)));

 //Subscribe to the PC and JointState topic, large queue size where all messages from the bag can fit in
  //-> adjust queue size later
  message_filters::Subscriber<sensor_msgs::PointCloud2> filtered_cloud(nh, "/points2", 150);
  message_filters::Subscriber<sensor_msgs::JointState> joint_states(nh, "/joint_states", 1100);

  //Initialize approximate time filter, to fit together PCs and joint states by time stamp
  //again a large queue size to make sure no messages get lost
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::JointState> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(150), filtered_cloud, joint_states);
  sync.registerCallback(boost::bind(&cloud_cb, _1, _2, boost::ref(nh)));
  ros::spin ();
}
