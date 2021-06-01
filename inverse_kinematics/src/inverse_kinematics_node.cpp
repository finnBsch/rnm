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
#include "forward_kin/get_endeffector.h"
#include "jacobian.h"
#include <eigen_conversions/eigen_msg.h>



#define PI 3.14159265

using namespace tf;
using namespace Eigen;


boost::array<double, 7> desiredJointAngles;
VectorXd jointAngles;

VectorXd currentA_vector(12, 1);
VectorXd finalA_vector(12, 1);

MatrixXd currentA;


float incrementalStepSize = 10;



/*
 * This function takes A 4x4 Matrix and converts it to a 1x12 vector
 *
 * @param: MatrixXd M
 * returns: VectorXd v
 */
VectorXd convert4DMatrixTo12DVector(MatrixXd M){
  VectorXd v(12);
  v << M(0,0), M(1,0), M(2,0),
      M(0,1), M(1,1), M(2,1),
      M(0,2), M(1,2), M(2,2),
      M(0,3), M(1,3), M(2,3);
  return v;
}



/*void callback(const sensor_msgs::JointState& msg) {
    // fill jointAngles vector with joinState entries
    for (int i = 0; i < 7; ++i) {
        jointAngles(i, 0) = msg.position[i];
    }
}*/


boost::array<double, 7> transformVectorToArray(VectorXd vector){
    return {vector[0], vector[1],vector[2],
            vector[3], vector[4],vector[5],
            vector[6]};
}

/*
 * This recursive function performs the incremental inverse kinematics. It takes the current joint angles, the current A and final A Matrix as 12x1 vectors. It returns the desired joint angles for the desired position of the robot
 *
 * @param: VectorXd thetas, VectorXd currentA. VectorXd finalA
 * returns: VectorXd desiredThetas
 */
boost::array<double, 7> incrementalStep(sensor_msgs::JointState joint_state_msg, forward_kin::get_endeffector srv,
ros::ServiceClient client){

    // --------------- GET JOINT ANGLES -------------------
    joint_state_msg  = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",ros::Duration(10)));
    jointAngles << joint_state_msg.position[0], joint_state_msg.position[1],joint_state_msg.position[2], joint_state_msg.position[3], joint_state_msg.position[4],joint_state_msg.position[5], joint_state_msg.position[6];


    // --------------- GET A MATRIX -------------------
    // check connection
    auto a = client.call(srv);
    if (a)
    {
        ROS_INFO("Endpos: %f", srv.response.layout);
    }
    else
    {
        ROS_ERROR("Failed to call service forward_kin");
        return 1;
    }

    // get the current A Matrix from the forward_kin node

    wrenchEigenToMsg(srv.response.layout, currentA); // alternative try: transformMsgToEigen and in Forward_kin_node.cpp: twistEigenToMsg

    // conversion of matrices to vectors
    currentA_vector = convert4DMatrixTo12DVector(currentA);


    //---------------- DO INCREMENTAL CALCULATION ----------------

    // calculate the difference
    VectorXd delta_A = (finalA_vector - currentA_vector);
    
    if (abs(delta_A.maxCoeff()) <= 0.01){
        return transformVectorToArray(jointAngles);
    } else{

    MatrixXd J = calculateJacobian(jointAngles);
    MatrixXd pinvJ = J.completeOrthogonalDecomposition().pseudoInverse();


    // calculate new joint angles
    VectorXd newJointAngles = jointAngles + pinvJ * delta_A/incrementalStepSize;


    // call forward kin node and transmit the new joint angles
        srv.request.joint_angles = transformVectorToArray(jointAngles);

    return incrementalStep(joint_state_msg, srv, client);
  }
}



int main(int argc, char** argv)  {


  // create inverse-kinematics node
  ros::init(argc,argv, "inverse_kinematics_node");
  ros::NodeHandle nodeHandle("~");


  //-------------------- Joint Angles ---------------------------
  // Parse parameters specified in the .launch file
  std::string topic_name;
  int queue_size;
  nodeHandle.getParam("topic_name", topic_name);
  nodeHandle.getParam("queue_size", queue_size);

  sensor_msgs::JointState joint_state_msg;


  // Register a callback function (a function that is called every time a new message arrives)
  //ros::Subscriber sub = nodeHandle.subscribe("/joint_states", 10, callback);


  //-------------------- Matrix A ------------------------------
  // create client and subscribe to service
  forward_kin::get_endeffector srv;
  ros::ServiceClient client = nodeHandle.serviceClient<forward_kin::get_endeffector>("forward_kin_node/get_endeffector");
  
  finalA_vector = currentA_vector * 0.90; // ONLY FOR TESTING

  desiredJointAngles = incrementalStep(joint_state_msg, srv, client);

  std::cout << desiredJointAngles[0] << desiredJointAngles[1];
  //PUBLISH





  // perform incremental steps
  // desiredThetas = incrementalStep(initialThetas, initialA_vector, desiredA_vector);

  // final Matrix A
  //MatrixXd finalA = exampleMatrix_A_03(desiredThetas[0], desiredThetas[1], desiredThetas[2]);
  ros::spin();
}

