/*
 * This is the example of Tutorial 5 to test and understand incremental inverse kinematics
 * author: Sean Maroofi
 * added: 29.5.21
 * last edited: 29.5.21
 */

#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include <eigen3/Eigen/Dense>
#include <stdlib.h>
#include "../../general/convert.h"
#include "../../forward_kin/srv/get_endeffector.h"


#define PI 3.14159265

using namespace Eigen;



/*
 * This recursive function performs the incremental inverse kinematics. It takes the current joint angles, the current A and final A Matrix as 12x1 vectors. It returns the desired joint angles for the desired position of the robot
 *
 * @param: VectorXd thetas, VectorXd currentA. VectorXd finalA
 * returns: VectorXd desiredThetas
 */


VectorXd incrementalStep(VectorXd currentThetas, VectorXd currentA, VectorXd finalA){
  VectorXd delta_A = (finalA - currentA);

  if (abs(delta_A.maxCoeff()) <= 0.01){
    return currentThetas;
  } else{
    MatrixXd J = jacobian(currentThetas[0], currentThetas[1], currentThetas[2]);
    MatrixXd pinvJ = J.completeOrthogonalDecomposition().pseudoInverse();

    VectorXd nextThetas = currentThetas + pinvJ * delta_A/10;
    VectorXd nextA = convert4DMatrixTo12DVector(exampleMatrix_A_03(nextThetas[0], nextThetas[1], nextThetas[2]));
    return incrementalStep(nextThetas, nextA, finalA);
  }
}


int main(int argc, char** argv)  {

  // matrices and vectors for the equations
  static VectorXd initialThetas(3);
  static VectorXd desiredThetas(3);
  static Matrix4Xd initialA;
  static Matrix4Xd desiredA;

  ros::init(argc,argv, "inverse_kinematics_node");
  ros::NodeHandle n;
  forward_kin::get_endeffector srv;
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg  = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",ros::Duration(10)));
  ros::ServiceClient client = n.serviceClient<forward_kin::get_endeffector>("forward_kin_node/get_endeffector");
  boost::array<double, 7> arr = {joint_state_msg.position[0], joint_state_msg.position[1],joint_state_msg.position[2],
                                   joint_state_msg.position[3], joint_state_msg.position[4],joint_state_msg.position[5],
                                   joint_state_msg.position[6]};
  srv.request.A = arr;
  auto a = client.call(srv);
  // check if service worked
  if (a)
  {
        ROS_INFO("Endpos: %f", srv.response.end_effector_pos[0]);
  }
  else
  {
        ROS_ERROR("Failed to call service forward_kin");
        return 1;
  }









  // initialize variables
  initialThetas << 40*PI/180, 40*PI/180, 20;


  initialA = nullptr;

  desiredA << 0.5792, -0.7071, 0.4056,  93.2834,
      0.5792,  0.7071, 0.4056,  93.2834,
      -0.5736,  0     , 0.8192, 288.4050,
      0     ,  0     , 0     ,   1;

  // Matrix to vectors transformation
  VectorXd initialA_vector = convert4DMatrixTo12DVector(initialA);
  VectorXd desiredA_vector = convert4DMatrixTo12DVector(desiredA);

  // perform incremental steps
  desiredThetas = incrementalStep(initialThetas, initialA_vector, desiredA_vector);

  // final Matrix A
  //MatrixXd finalA = exampleMatrix_A_03(desiredThetas[0], desiredThetas[1], desiredThetas[2]);

}

