///*
// * This is the example of Tutorial 5 to test and understand incremental inverse kinematics
// * author: Sean Maroofi
// * added: 25.5.21
// * last edited: 26.5.21
// */
//#include <ros/ros.h>
//#include "sensor_msgs/JointState.h"
//#include <eigen3/Eigen/Dense>
//#include <stdlib.h>
//
//#define PI 3.14159265
//
//using namespace Eigen;
//
//// matrices and vectors for the equations
//static VectorXd initialThetas(3);
//static VectorXd desiredThetas(3);
//static Matrix4Xd initialA;
//static Matrix4Xd desiredA;
//
//// length of robot arm
//int l1 = 100;
//int l2 = 50;
//int l3 = 150;
//
///*
// * This function returns the A Matrix of the example of Tutorial 5. It takes 3 joint angles as input parameters
// *
// * @param: float theta1, float theta2, float d3*
// * returns: Matrix A
// */
//MatrixXd exampleMatrix_A_03(float theta1, float theta2, float d3){
//  MatrixXd A(4,4);
//  A <<  cos(theta1)*cos(theta2), -sin(theta1), cos(theta1)*sin(theta2), (l2 + l3 + d3)*cos(theta1)*sin(theta2),
//        cos(theta2)*sin(theta1), cos(theta1) , sin(theta1)*sin(theta2), (l2 + l3 + d3)*sin(theta1)*sin(theta2),
//        -sin(theta2)           , 0           , cos(theta2)            , l1 + (l2 + l3 + d3)*cos(theta2)       ,
//        0                      , 0           , 0                      , 1;
//
//  return A;
//}
//
////-----------------already outsourced in convert.h----------------------------
////
//// * This function takes A 4x4 Matrix and converts it to a 1x12 vector
//// *
//// * @param: MatrixXd M
//// * returns: VectorXd v
//// */
////VectorXd convert4DMatrixTo12DVector(MatrixXd M){
////  VectorXd v(12);
////    v << M(0,0), M(1,0), M(2,0),
////         M(0,1), M(1,1), M(2,1),
////         M(0,2), M(1,2), M(2,2),
////         M(0,3), M(1,3), M(2,3);
////  return v;
////}
//
///*
// * This function gives the Jacobian of the example Matrix A. It takes the joint angles and returns a 12x3 Matrix.
// *
// * @param: float theta1, float theta2, float d3*
// * returns: MatrixXd Jacobian
// */
//MatrixXd jacobian (float theta1, float theta2, float d3){
//  MatrixXd Jacobi(12, 3);
//                  Jacobi <<  -sin(theta1)*cos(theta2)               , -sin(theta2)*cos(theta1)              , 0,
//                             cos(theta1)*cos(theta2)                , -sin(theta1)*sin(theta2)              , 0,
//                             0                                      , -cos(theta2)                          , 0,
//                             -cos(theta1)                           , 0                                     , 0,
//                             -sin(theta1)                           , 0                                     , 0,
//                             0                                      , 0                                     , 0,
//                             -sin(theta1)*sin(theta2)               , cos(theta1)*cos(theta2)               , 0,
//                             sin(theta2)*cos(theta1)                , sin(theta1)*cos(theta2)               , 0,
//                             0                                      , -sin(theta2)                          , 0,
//                             -(d3 + l2 + l3)*sin(theta1)*sin(theta2), (d3 + l2 +l3)*cos(theta1)*cos(theta2) , sin(theta2)*cos(theta1),
//                             (d3 + l2 + l3)*sin(theta2)*cos(theta1) , (d3 + l2 + l3)*sin(theta1)*cos(theta2), sin(theta1)*sin(theta2),
//                             0                                      , -(d3 + l2 + l3)*sin(theta2)           , cos(theta2);
//  return Jacobi;
//}
//
///*
// * This recursive function performs the incremental inverse kinematics. It takes the current joint angles, the current A and final A Matrix as 12x1 vectors. It returns the desired joint angles for the desired position of the robot
// *
// * @param: VectorXd thetas, VectorXd currentA. VectorXd finalA
// * returns: VectorXd desiredThetas
// */
//VectorXd incrementalStep(VectorXd currentThetas, VectorXd currentA, VectorXd finalA){
//  VectorXd delta_A = (finalA - currentA);
//
//  if (abs(delta_A.maxCoeff()) <= 0.01){
//      return currentThetas;
//    } else{
//      MatrixXd J = jacobian(currentThetas[0], currentThetas[1], currentThetas[2]);
//      MatrixXd pinvJ = J.completeOrthogonalDecomposition().pseudoInverse();
//
//      VectorXd nextThetas = currentThetas + pinvJ * delta_A/10;
//      VectorXd nextA = convert4DMatrixTo12DVector(exampleMatrix_A_03(nextThetas[0], nextThetas[1], nextThetas[2]));
//      return incrementalStep(nextThetas, nextA, finalA);
//    }
//}
//
//int main(int argc, char** argv)  {
//  // initialize variables
//  initialThetas << 40*PI/180, 40*PI/180, 20;
//  initialA = exampleMatrix_A_03(initialThetas[0],initialThetas[1], initialThetas[2]);
//  desiredA << 0.5792, -0.7071, 0.4056,  93.2834,
//              0.5792,  0.7071, 0.4056,  93.2834,
//             -0.5736,  0     , 0.8192, 288.4050,
//              0     ,  0     , 0     ,   1;
//
//  // Matrix to vectors transformation
//  VectorXd initialA_vector = convert4DMatrixTo12DVector(initialA);
//  VectorXd desiredA_vector = convert4DMatrixTo12DVector(desiredA);
//
//  // perform incremental steps
//  desiredThetas = incrementalStep(initialThetas, initialA_vector, desiredA_vector);
//
//  // final Matrix A
//  MatrixXd finalA = exampleMatrix_A_03(desiredThetas[0], desiredThetas[1], desiredThetas[2]);
//
//}