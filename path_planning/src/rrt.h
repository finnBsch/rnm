//
// Created by finn on 27.05.21.
//

#ifndef SRC_RRT_H
#define SRC_RRT_H
#include "rrt_node.h"
#include <tuple>
#include "ros/ros.h"
#include <flann/algorithms/dist.h>
#include <flann/algorithms/kdtree_single_index.h>
#include <flann/flann.hpp>
#include <eigen3/Eigen/Dense>
#include <boost/math/interpolators/cubic_b_spline.hpp>
#include <eigen3/Eigen/SVD>
#include "bullet/btBulletCollisionCommon.h"
#include "inverse_kinematics/unserService.h"
typedef vector<vector<vector<vector<rrt_node*>>>> node_grid;
using namespace Eigen;
class rrt {
 private:
  // new goal point stuff
  ros::ServiceClient* client_inv;
  inverse_kinematics::unserService srv_inv;
  // collision stuff
  int ob_count = 0;
  btCollisionWorld* mColWorld;
  btDefaultCollisionConfiguration*        mConfig;
  btCollisionDispatcher*                  mDispatcher;
  btBroadphaseInterface*                  mBroadphase;
  btAlignedObjectArray<btCollisionObject*> mObjects;
  btAlignedObjectArray<btCollisionObject*> mColObjects;
  void initialize_world();
  btCollisionObject* add_cylinder(btVector3 vect,  int orient=1);
  btCollisionObject* add_body_obstacle(float height);
  btCollisionObject* add_sphere(float radius);
  bool check_collision(joint_angles angles);
  // informed rrt
  float c_opt;
  Matrix<float, 6, 1> center;
  Matrix<float, 6, 6> C;
  Matrix<float, 6, 6> L;
  // kinematics
  Matrix<float, 8, 1> a;
  Matrix<float, 8, 1> d;
  Matrix<float, 8, 1> alpha;
  // Flann
  flann::Index<flann::L2_Simple<float>> kdtree;
  vector<rrt_node*> all_nodes;
  joint_angles goal_point;
  joint_angles goal_point_found;
  bool goal_found = false;
  Point goal_p;
  joint_angles start_point;
  rrt_node* start_node;
  Point goal_normal = {0, 0, -1};
  // RRT Params
  rrt_params params;
  // TODO save obstacles
  rrt_node* findNearestNode(joint_angles& relative_to);
  vector<rrt_node*> findNearNodes(joint_angles& relative_to);
  Point get_end_effector(joint_angles angles);
  Point get_end_effector_normal(joint_angles angles);
  joint_angles sample_ellipsoid();
  joint_angles sample_intelligent();
  void calculateC(joint_angles gp);

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  long long num_nodes = 0;
  long long nodesmark_goal_found = 0;
  float min_dist = -1;
  float min_dist_orient = -1;
  rrt_node* goal_node = nullptr;
  rrt(joint_angles start_point, joint_angles goal_point,rrt_params params, ros::ServiceClient* client_inv,     inverse_kinematics::unserService srv_inv);
  tuple<bool, array<Point, 2>> expand();
  vector<tuple<Point, joint_angles>> return_goal_path();

};
struct SimulationContactResultCallback : public btCollisionWorld::ContactResultCallback
{

  bool bCollision;

  SimulationContactResultCallback() : bCollision(false)
  {}

  btScalar addSingleResult(btManifoldPoint& cp,
                           const btCollisionObjectWrapper* colObj0Wrap,
                           int partId0,
                           int index0,
                           const btCollisionObjectWrapper* colObj1Wrap,
                           int partId1,
                           int index1)
  {
    //If cp distance less than threshold

    bCollision=true;
    return 0;
  }
};
#endif  // SRC_RRT_H
