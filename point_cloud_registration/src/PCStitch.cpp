#include <iostream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
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
#include <eigen_conversions/eigen_msg.h>
#include "forward_kin/get_endeffector.h"
#include "point_cloud_registration/PCJScombined.h"
#include "point_cloud_registration/alignment_service.h"
#include "point_cloud_registration/registration_results_service.h"

class PCStitch
{
 public:
  PCStitch(ros::NodeHandle& nh, Eigen::Matrix4f handeye):
        nh_ (nh),
        handeye_ (handeye),
        pub_ (nh.advertise<sensor_msgs::PointCloud2> ("stitched_cloud", 1)),
        message_counter_(0)
  {
    stitched_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB> ());
    cloud_xyz_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB> ());
  }

 private:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr stitched_cloud_;
  pcl::PCLPointCloud2* received_cloud_{};
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyz_;
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  sensor_msgs::JointState joint_states_;
  Eigen::Matrix4f transformation_matrix_;
  Eigen::Matrix4f handeye_;
  int message_counter_;

  const int message_limit = 11;
  const float leaf_size = 0.005;
  const Eigen::Vector4f xyz_min = {0.15, -0.5, 0.02, 1.0};
  const Eigen::Vector4f xyz_max = {0.7, 0.5, 0.3, 1.0};
  const int mean_k = 50;
  const double std_dev_max = 1;
  const int icp_max = 50;
  const float box_length = 0.73;
  const float box_width = 0.355;
  const float box_height = 0.27;
  const Eigen::Vector3f box_centerpoint = {-32.5/1000, -65/1000, 5/1000};
  const Eigen::Vector4f needle_startpoint = {-30.0/1000, -100.674/1000, 160.0/1000, 1.0};
  const Eigen::Vector4f needle_goalpoint = {-4.0/1000, -21.0/1000, 22.0/1000, 1.0};
  const Eigen::Vector4f skeleton_centerpoint =  {-32.5/1000, -65/1000, 5/1000, 1.0};



  void receiveCloud(const point_cloud_registration::PCJScombined::ConstPtr& PCJS){
    received_cloud_ = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(PCJS->PC, *received_cloud_);
    joint_states_ = PCJS->JS;
    pcl::fromPCLPointCloud2(*received_cloud_, *cloud_xyz_);
    message_counter_++;
  }

  void subsample(){
    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (cloud_xyz_);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*cloud_xyz_);

  }

  void outlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& outlierCloud){
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(outlierCloud);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(std_dev_max);
    sor.filter(*outlierCloud);
  }

  void transform(){
    forward_kin::get_endeffector srv;

    ros::ServiceClient client = nh_.serviceClient<forward_kin::get_endeffector>("forward_kin_node/get_endeffector");
    boost::array<double, 7> angles = {joint_states_.position[0], joint_states_.position[1],joint_states_.position[2],
                                      joint_states_.position[3], joint_states_.position[4],joint_states_.position[5],
                                      joint_states_.position[6]};

    srv.request.joint_angles = angles;

    // --------------- GET A MATRIX -------------------
    client.call(srv);

    // get the current Transformation matrix Matrix from the forward_kin node
    Eigen::Matrix4f transform;
    transform << srv.response.layout.data[0],srv.response.layout.data[1],srv.response.layout.data[2],srv.response.layout.data[3],
        srv.response.layout.data[4],srv.response.layout.data[5],srv.response.layout.data[6],srv.response.layout.data[7],
        srv.response.layout.data[8],srv.response.layout.data[9],srv.response.layout.data[10],srv.response.layout.data[11],
        srv.response.layout.data[12],srv.response.layout.data[13],srv.response.layout.data[14],srv.response.layout.data[15];
    transform = transform*handeye_;
    
    //Transformation of the filtered cloud
    pcl::transformPointCloud (*cloud_xyz_, *cloud_xyz_, transform);
  }

  void crop(){
    pcl::CropBox<pcl::PointXYZRGB> crop;
    crop.setMin(Eigen::Vector4f(xyz_min));
    crop.setMax(Eigen::Vector4f(xyz_max));
    crop.setInputCloud(cloud_xyz_);
    crop.filter(*cloud_xyz_);
  }

  void ICP(){
    if (stitched_cloud_->empty()){
      *stitched_cloud_ = *cloud_xyz_;
    }else{
      //Set clouds for the ICP algorithm
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr goal_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
      pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
      icp.setInputSource(cloud_xyz_);
      icp.setInputTarget(stitched_cloud_);
      icp.setMaximumIterations(icp_max);

      //align new cloud to stitched cloud and add to the total stitched cloud.
      icp.align(*goal_cloud);
      *stitched_cloud_ += *goal_cloud;
    }
  }

  void publishCloud(){
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*stitched_cloud_, output);
    pub_.publish (output);
  }

  void saveCloud(){
    pcl::io::savePCDFile("/home/niklas/Documents/RNM/stitched_cloud.pcd", *stitched_cloud_, true);
  }

  void calculateSkeletonPosition() {
    point_cloud_registration::alignment_service srv;

    ros::ServiceClient client = nh_.serviceClient<point_cloud_registration::alignment_service>(
        "STLRegistration2/alignment_service");
    pcl::toROSMsg(*stitched_cloud_, srv.request.stitched_cloud);
    client.call(srv);

    transformation_matrix_ << srv.response.alignment_transformation.data[0],
        srv.response.alignment_transformation.data[1],
        srv.response.alignment_transformation.data[2],
        srv.response.alignment_transformation.data[3],
        srv.response.alignment_transformation.data[4],
        srv.response.alignment_transformation.data[5],
        srv.response.alignment_transformation.data[6],
        srv.response.alignment_transformation.data[7],
        srv.response.alignment_transformation.data[8],
        srv.response.alignment_transformation.data[9],
        srv.response.alignment_transformation.data[10],
        srv.response.alignment_transformation.data[11],
        srv.response.alignment_transformation.data[12],
        srv.response.alignment_transformation.data[13],
        srv.response.alignment_transformation.data[14],
        srv.response.alignment_transformation.data[15];
    std::cerr << transformation_matrix_ << std::endl;
  }

  bool getRegistrationResults(point_cloud_registration::registration_results_service::Request& req, point_cloud_registration::registration_results_service::Response &res){
    res.registration_results.box_height = box_height;
    res.registration_results.box_length = box_length;
    res.registration_results.box_width = box_width;
    tf::matrixEigenToMsg(transformation_matrix_*needle_startpoint, res.registration_results.needle_startpoint);
    tf::matrixEigenToMsg(transformation_matrix_*needle_goalpoint, res.registration_results.needle_goalpoint);
    tf::matrixEigenToMsg(transformation_matrix_*skeleton_centerpoint, res.registration_results.skeleton_centerpoint);
    tf::matrixEigenToMsg(transformation_matrix_, res.registration_results.registration_transformation);
    return true;
  }

  void startService(){
    ros::ServiceServer service = nh_.advertiseService("registration_results_service", &PCStitch::getRegistrationResults, this);
    ros::spin();
  }

 public:
  void addCloud(const point_cloud_registration::PCJScombined::ConstPtr& PCJS){
    receiveCloud(PCJS);
    transform();
    crop();
    outlierRemoval(cloud_xyz_);
    subsample();
    ICP();
    publishCloud();
    if (message_counter_ == message_limit){
      outlierRemoval(stitched_cloud_);
      publishCloud();
      saveCloud();
      calculateSkeletonPosition();
      startService();
    }
  }
};

int
main (int argc, char** argv)
{
  const int queue_size = 50;


  ros::init (argc, argv, "PCStitch");
  ros::NodeHandle nh;

  Eigen::Matrix4f handeye;
  handeye << -0.0342177, -0.0224303,   0.999141,  0.0357424,
      -0.998979, -0.0285611,  -0.034854, -0.0232153,
      0.0293307,  -0.999324, -0.0214294,  0.0547717,
      0,          0,          0,          1;

  PCStitch pcs(nh, handeye);

  ros::Subscriber sub = nh.subscribe<point_cloud_registration::PCJScombined>("/PCJScombined", queue_size, &PCStitch::addCloud, &pcs);

  ros::spin();
}

