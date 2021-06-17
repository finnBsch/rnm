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

class FeatureCloud
{
 public:
  // A bit of shorthand
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
  typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
  typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
  typedef pcl::search::KdTree<pcl::PointXYZRGB> SearchMethod;

  FeatureCloud () :
      search_method_xyz_ (new SearchMethod),
      normal_radius_ (0.02f),
      feature_radius_ (0.02f)
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
  // Compute the surface normals and local features
  void
  processInput ()
  {
    voxelGridFiltering();
    computeSurfaceNormals ();
    computeLocalFeatures ();
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
  computeSurfaceNormals ()
  {
    normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
    norm_est.setInputCloud (xyz_);
    norm_est.setSearchMethod (search_method_xyz_);
    norm_est.setRadiusSearch (normal_radius_);
    norm_est.compute (*normals_);
  }

  // Compute the local feature descriptors
  void
  computeLocalFeatures ()
  {
    features_ = LocalFeatures::Ptr (new LocalFeatures);

    pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud (xyz_);
    fpfh_est.setInputNormals (normals_);
    fpfh_est.setSearchMethod (search_method_xyz_);
    fpfh_est.setRadiusSearch (feature_radius_);
    fpfh_est.compute (*features_);
  }

 private:
  // Point cloud data
  PointCloud::Ptr xyz_;
  SurfaceNormals::Ptr normals_;
  LocalFeatures::Ptr features_;
  SearchMethod::Ptr search_method_xyz_;

  // Parameters
  float normal_radius_;
  float feature_radius_;
  float leaf_size = 0.005;
};


//---------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------

class TemplateAlignment {
 public:
  // A struct for storing alignment results
  struct Result {
    float fitness_score_coarse;
    float fitness_score_icp;
    Eigen::Matrix4f final_transformation_coarse;
    Eigen::Matrix4f final_transformation_ICP;
    pcl::PointCloud<pcl::PointXYZRGB> skeleton_cloud_tf;

    // PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  TemplateAlignment()
      : min_sample_distance_(0.005f),                   // 0.05
        max_correspondence_distance_(0.002f * 0.002f),  // 0.01f*0.01f
        nr_iterations_(500) {
    // Initialize the parameters in the Sample Consensus Initial Alignment (SAC-IA) algorithm
    sac_ia_.setMinSampleDistance(min_sample_distance_);
    sac_ia_.setMaxCorrespondenceDistance(max_correspondence_distance_);
    sac_ia_.setMaximumIterations(nr_iterations_);
  }

  ~TemplateAlignment() {}

  // Set the given cloud as the target to which the templates will be aligned
  void setTargetCloud(FeatureCloud& target_cloud) {
    target_ = target_cloud;
    sac_ia_.setInputTarget(target_cloud.getPointCloud());
    sac_ia_.setTargetFeatures(target_cloud.getLocalFeatures());
  }

  /*
  // Add the given cloud to the list of template clouds
  void
  addTemplateCloud (FeatureCloud &template_cloud)
  {
    templates_.push_back (template_cloud);
  }
   */

  // Align the given template cloud to the target specified by setTargetCloud ()
  void align(FeatureCloud& template_cloud, TemplateAlignment::Result& result) {
    sac_ia_.setInputSource(template_cloud.getPointCloud());
    sac_ia_.setSourceFeatures(template_cloud.getLocalFeatures());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr registration_output(new pcl::PointCloud<pcl::PointXYZRGB>());
    sac_ia_.align(*registration_output);

    result.fitness_score_coarse = (float)sac_ia_.getFitnessScore(max_correspondence_distance_);
    result.final_transformation_coarse = sac_ia_.getFinalTransformation();

    icp.setInputSource(registration_output);
    icp.setInputTarget(target_.getPointCloud());

    // ICP parameters (examples), we still need to adjust them
    // icp.setEuclideanFitnessEpsilon(1);
    // icp.setTransformationEpsilon(1e-20);
    icp.setMaximumIterations(20);

    // align new cloud to stitched cloud and add to the total stitched cloud.
    icp.align(*registration_output);
    result.fitness_score_icp = icp.getFitnessScore();
    result.final_transformation_ICP = icp.getFinalTransformation();
    result.skeleton_cloud_tf = *registration_output;
  }

  /*
  // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud
  () void alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> >
  &results)
  {
    results.resize (templates_.size ());
    for (std::size_t i = 0; i < templates_.size (); ++i)
    {
      align (templates_[i], results[i]);
    }
  }

  // Align all of template clouds to the target cloud to find the one with best alignment score
  int
  findBestAlignment (TemplateAlignment::Result &result)
  {
    // Align all of the templates to the target cloud
    std::vector<Result, Eigen::aligned_allocator<Result> > results;
    alignAll (results);

    // Find the template with the best (lowest) fitness score
    float lowest_score = std::numeric_limits<float>::infinity ();
    int best_template = 0;
    for (std::size_t i = 0; i < results.size (); ++i)
    {
      const Result &r = results[i];
      if (r.fitness_score < lowest_score)
      {
        lowest_score = r.fitness_score;
        best_template = (int) i;
      }
    }

    // Output the best alignment
    result = results[best_template];
    return (best_template);
  }
*/

 private:
  // A list of template clouds and the target to which they will be aligned
  std::vector<FeatureCloud> templates_;
  FeatureCloud target_;

  // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
  pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> sac_ia_;
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  float min_sample_distance_;
  float max_correspondence_distance_;
  int nr_iterations_;
};


// ------------------------------------------------------------------------------------------------------
int main(int argc, char** argv) {
  ros::init(argc, argv, "STLRegistration");
  ros::NodeHandle nodeHandle;
  ros::Publisher publisher;
  publisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("skeleton_cloud_tf", 1);
  ros::Publisher publisher_stitch;
  publisher_stitch = nodeHandle.advertise<sensor_msgs::PointCloud2>("stitched_cloud", 1);

  const std::string STL_file_name = "/home/niklas/Documents/RNM/Scanning/Skeleton.stl";

  FeatureCloud template_cloud;
  template_cloud.loadfromSTL(STL_file_name);

  // load stitched point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr stitched_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::io::loadPCDFile("/home/niklas/Documents/RNM/stitched_cloud.pcd", *stitched_cloud);

  // Assign to the target FeatureCloud
  FeatureCloud target_cloud;
  target_cloud.setInputCloud(stitched_cloud);

  TemplateAlignment template_align;
  TemplateAlignment::Result alignment_results;
  template_align.setTargetCloud (target_cloud);
  template_align.align(template_cloud,alignment_results);


  // Print the alignment fitness score (values less than 0.00002 are good)
  printf ("Fitness score coarse alignment: %f\n", alignment_results.fitness_score_coarse);
  printf ("Fitness score ICP alignment: %f\n", alignment_results.fitness_score_icp);


  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(alignment_results.skeleton_cloud_tf, output);
  output.header.frame_id = "rgb_camera_link";
  publisher.publish (output);
  sensor_msgs::PointCloud2 output_stitch;
  pcl::toROSMsg(*stitched_cloud, output_stitch);
  output_stitch.header.frame_id = "rgb_camera_link";
  publisher_stitch.publish (output_stitch);

}
