#include <ros/ros.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <std_msgs/Float64MultiArray.h>
#include "point_cloud_registration/alignment_service.h"
#include <eigen_conversions/eigen_msg.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>

class FeatureCloud
{
 public:

  typedef pcl::PointXYZRGBNormal PointNT;
  typedef pcl::PointCloud<PointNT> PointCloudT;
  typedef pcl::FPFHSignature33 FeatureT;
  typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
  typedef pcl::PointCloud<FeatureT> FeatureCloudT;

  float leaf_size_;

  FeatureCloud () :
      normal_radius_ (0.01f),
      feature_radius_ (0.025f),
      leaf_size_(0.005f)
  {}

  ~FeatureCloud () {}

  // Process the given cloud
  void
  setInputCloud (PointCloudT::Ptr xyz)
  {
    xyz_ = xyz;
    processInput ();
  }

  // Load and process the cloud in the given PCD file
  void
  loadfromSTL (const std::string &STL_file)
  {
    xyz_ = PointCloudT::Ptr (new PointCloudT);
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
  PointCloudT::Ptr
  getPointCloud () const
  {
    return (xyz_);
  }


  // Get a pointer to the cloud of feature descriptors
  FeatureCloudT::Ptr
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
    pcl::VoxelGrid<pcl::PointXYZRGBNormal> vg;
    vg.setInputCloud(xyz_);
    vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
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
  featureEstimation ()
  {
    features_ = FeatureCloudT::Ptr (new FeatureCloudT);
    pcl::console::print_highlight ("Estimating features...\n");
    FeatureEstimationT fest;
    fest.setRadiusSearch (feature_radius_);
    fest.setInputCloud (xyz_);
    fest.setInputNormals (xyz_);
    fest.compute (*features_);
  }

 private:
  // Point cloud data
  PointCloudT::Ptr xyz_;
  FeatureCloudT::Ptr features_;

  // Parameters
  float normal_radius_;
  float feature_radius_;
};

class CloudAlignment {
 public:

  typedef pcl::PointXYZRGBNormal PointNT;
  typedef pcl::PointCloud<PointNT> PointCloudT;
  typedef pcl::FPFHSignature33 FeatureT;
  // For storing alignment results
  struct Result {
    float fitness_score_coarse;
    float fitness_score_icp;
    Eigen::Matrix4f final_transformation_coarse;
    Eigen::Matrix4f final_transformation_ICP;
    PointCloudT skeleton_cloud_tf;

  };

  CloudAlignment()
      :

        nr_iterations_(50000),
        number_of_samples_(3),
        k_value_(5),
        similarity_threshold_(0.8f),
        max_correspondence_distance_(2.5*0.005),
        inlier_fraction_(0.6f),
        nr_iterations_icp_(50)
  {
    // Input parameters in alignment algorithm
    align_.setMaximumIterations(nr_iterations_);
    align_.setNumberOfSamples(number_of_samples_);
    align_.setCorrespondenceRandomness(k_value_);
    align_.setSimilarityThreshold(similarity_threshold_);
    align_.setMaxCorrespondenceDistance(max_correspondence_distance_);
    align_.setInlierFraction(inlier_fraction_);
  }

  ~CloudAlignment() {}

  //Define the target cloud to which the source cloud will be aligned
  void setTargetCloud(FeatureCloud& target_cloud) {
    target_ = target_cloud;
    align_.setInputTarget(target_cloud.getPointCloud());
    align_.setTargetFeatures(target_cloud.getLocalFeatures());
  }

  // Align the given template cloud to the target specified by setTargetCloud ()
  void align(FeatureCloud& template_cloud, CloudAlignment::Result& result) {
    align_.setInputSource(template_cloud.getPointCloud());
    align_.setSourceFeatures(template_cloud.getLocalFeatures());
    PointCloudT::Ptr registration_output(new PointCloudT);
    align_.align(*registration_output);

    //result.fitness_score_coarse = (float)align_.getFitnessScore(max_correspondence_distance_);
    result.final_transformation_coarse = align_.getFinalTransformation();

    if (align_.hasConverged ())
    {
      //pcl::IterativeClosestPoint<PointNT , PointNT> icp;
      icp.setInputSource(registration_output);
      icp.setInputTarget(target_.getPointCloud());

      // ICP parameters (examples), we still need to adjust them
      icp.setMaximumIterations(nr_iterations_icp_);

      // align new cloud to stitched cloud and add to the total stitched cloud.
      icp.align(*registration_output);
      result.fitness_score_icp = icp.getFitnessScore();
      result.final_transformation_ICP = icp.getFinalTransformation();
      result.skeleton_cloud_tf = *registration_output;
    }
    else
    {
      pcl::console::print_error ("Alignment failed!\n");
    }
  }

 private:
  // A list of template clouds and the target to which they will be aligned

  FeatureCloud target_;

 pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align_;
  pcl::IterativeClosestPoint<PointNT, PointNT> icp;
  int nr_iterations_;
  float number_of_samples_;
  float k_value_;
  float similarity_threshold_;
  float max_correspondence_distance_;
  float inlier_fraction_;
  int nr_iterations_icp_;
};
// ------------------------------------------------------------------------------------------------------

  class AlignService
{
 public:
  AlignService(ros::NodeHandle& nodeHandle){
    nh_ = nodeHandle;
    pub_ = nodeHandle.advertise<sensor_msgs::PointCloud2>("/skeleton_cloud_tf", 1);

  }
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  bool alignmentService(point_cloud_registration::alignment_service::Request& req,
                        point_cloud_registration::alignment_service::Response& res) {

    const std::string STL_file_name =  "/home/niklas/Documents/RNM/Scanning/Skeleton.stl";

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr stitched_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::PCLPointCloud2 stitched_cloud_PC2;
    pcl_conversions::toPCL(req.stitched_cloud, stitched_cloud_PC2);
    pcl::fromPCLPointCloud2(stitched_cloud_PC2, *stitched_cloud);

    FeatureCloud skeleton_scanned;
    skeleton_scanned.setInputCloud(stitched_cloud);

    FeatureCloud skeleton_stl;
    skeleton_stl.loadfromSTL(STL_file_name);

    CloudAlignment alignment;
    CloudAlignment::Result alignment_results;
    alignment.setTargetCloud(skeleton_scanned);

    {
      pcl::ScopeTime t("Alignment");
      alignment.align (skeleton_stl,alignment_results);
    }

    Eigen::Matrix4f final_transformation;
    final_transformation =alignment_results.final_transformation_ICP * alignment_results.final_transformation_coarse;
    tf::matrixEigenToMsg(final_transformation, res.alignment_transformation);
    std::cout << res.alignment_transformation;

    // Print the alignment fitness score (values less than 0.00002 are good)
    printf("Fitness score coarse alignment: %f\n", alignment_results.fitness_score_coarse);
    printf("Fitness score ICP alignment: %f\n", alignment_results.fitness_score_icp);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(alignment_results.skeleton_cloud_tf, output);
    output.header.frame_id = "rgb_camera_link";
    pub_.publish(output);
    return true;
  }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "STLRegistration_node");
  ros::NodeHandle nodeHandle("~");

  AlignService Alignment(nodeHandle);
  // Register a callback function (a function that is called every time a new message arrives)
  ros::ServiceServer service = nodeHandle.advertiseService("alignment_service", &AlignService::alignmentService,&Alignment);
  ros::spin();

}
