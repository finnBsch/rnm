#include <ros/ros.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <thread>
#include <pcl/common/angles.h> // for pcl::deg2rad
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
//#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
//#include <pcl/registration/ndt.h>
#include <std_msgs/Float64MultiArray.h>
#include "point_cloud_registration/alignment_service.h"
#include <eigen_conversions/eigen_msg.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>

class FeatureCloud
{
 public:
  // A bit of shorthand
  typedef pcl::PointXYZRGBNormal PointNT;
  typedef pcl::PointCloud<PointNT> PointCloudT;
  typedef pcl::FPFHSignature33 FeatureT;
  typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
  typedef pcl::PointCloud<FeatureT> FeatureCloudT;
  typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

  FeatureCloud () :
      normal_radius_ (0.01f),
      feature_radius_ (0.025f)
  {}

  ~FeatureCloud () {}

  // Process the given cloud
  void
  setInputCloud (PointCloud::Ptr xyz)
  {
    xyz_ = xyz;
    processInput ();
  }

  // Load and process the cloud in the given PCD file
  void
  loadfromSTL (const std::string &STL_file)
  {
    xyz_ = PointCloud::Ptr (new PointCloud);
    pcl::PolygonMesh mesh;
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    pcl::io::loadPolygonFileSTL( STL_file,mesh);
    pcl::io::mesh2vtk(mesh, polydata); //Convert PolygonMesh object into vtkPolyData object
    pcl::io::vtkPolyDataToPointCloud(polydata, *xyz_);
    for (int i = 0; i < xyz_->points.size(); i++)
    {
      xyz_->points[i].x /=1000 ;
      xyz_->points[i].y /=1000 ;
      xyz_->points[i].z /=1000 ;
    }
    processInput ();
  }

  // Get a pointer to the cloud 3D points
  PointCloud::Ptr
  getPointCloud () const
  {
    return (xyz_);
  }

  // Get a pointer to the cloud of 3D surface normals
  SurfaceNormals::Ptr
  getSurfaceNormals () const
  {
    return (normals_);
  }

  // Get a pointer to the cloud of feature descriptors
  LocalFeatures::Ptr
  getLocalFeatures () const
  {
    return (features_);
  }

 protected:
  void
  processInput ()
  {
    voxelGridFiltering();
    normalEstimation ();
    featureEstimation ();
  }


  //Perform voxel grid filtering
  void
  voxelGridFiltering() {
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(xyz_);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*xyz_);
  }


  // Compute the surface normals
  void
  normalEstimation ()
  {
    pcl::console::print_highlight ("Estimating scene normals...\n");
    pcl::NormalEstimationOMP<PointNT,PointNT> norm_est;
    norm_est.setRadiusSearch (normal_radius_);
    norm_est.setInputCloud (xyz_);
    norm_est.compute (*xyz_);
  }

  // Compute the local feature descriptors
  void
  computeLocalFeatures ()
  {
    pcl::console::print_highlight ("Estimating features...\n");
    FeatureEstimationT fest;
    fest.setRadiusSearch (feature_radius_);
    fest.setInputCloud (xyz_);
    fest.setInputNormals (xyz_);
    fest.compute (*features_);
  }

 private:
  // Point cloud data
  PointCloud::Ptr xyz_;
  FeatureEstimationT features_;

  // Parameters
  float normal_radius_;
  float feature_radius_;
  float leaf_size = 0.005;
};
