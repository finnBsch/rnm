//
// Created by rnm on 26.06.21.
//

#include "forward_kinematics.h"
#include <eigen3/Eigen/Dense>
#include <opencv2/core/mat.hpp>

using namespace Eigen;
using namespace cv;




  /*
    * This function return the 4x4 Transformationmatrix for the given Denavit Hagen Parameters
    *
    * @param: float theta, float a, float d, float alpha
    * returns: MatrixXd A
   */
  MatrixXd Forward_kinematics::get_transformationmatrix(const float theta,
                                    const float a,
                                    const float d,
                                    const float alpha) {
    Matrix4d ret_mat;
    ret_mat << cos(theta), -sin(theta), 0, a, sin(theta) * cos(alpha), cos(theta) * cos(alpha),
        -sin(alpha), -d * sin(alpha), sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha),
        d * cos(alpha), 0, 0, 0, 1;
    return ret_mat;
  }

  /*
    * This function performs the Forward kinematics and returns the overall transformation matrix to the goal position as a vector
    *
    * @param: -
    * returns: VectorXd v
   */
  VectorXd Forward_kinematics::get_forward_kinematics_transformation_vector(VectorXd a,
                                                        VectorXd joint_angles_,
                                                        VectorXd d,
                                                        VectorXd alpha) {
    std::array<MatrixXd, 8> a_;
    Matrix4d A_total;
    VectorXd vector;

    for (int i = 0; i < 7; i++) {
      a_.at(i) = get_transformationmatrix(joint_angles_(i), a(i), d(i), alpha(i));
    }
    a_.at(7) = get_transformationmatrix(0, a(7), d(7), alpha(7));
    A_total = a_.at(0) * a_.at(1) * a_.at(2) * a_.at(3) * a_.at(4) * a_.at(5) * a_.at(6) * a_.at(7);

    vector << A_total(0, 0), A_total(1, 0), A_total(2, 0), A_total(0, 1), A_total(1, 1),
        A_total(2, 1), A_total(0, 2), A_total(1, 2), A_total(2, 2), A_total(0, 3), A_total(1, 3),
        A_total(2, 3);

    return vector;
  }

  std::tuple<Mat, Mat> Forward_kinematics::get_forward_kinematics_transformation(VectorXd a,
                                                                    VectorXd joint_angles_,
                                                                    VectorXd d,
                                                                    VectorXd alpha) {
    std::array<MatrixXd, 8> a_;
    Matrix4d A_total;
    Mat R_gripper2base;
    Mat t_gripper2base;

    for (int i = 0; i < 7; i++) {
      a_.at(i) = get_transformationmatrix(joint_angles_(i), a(i), d(i), alpha(i));
    }
    a_.at(7) = get_transformationmatrix(0, a(7), d(7), alpha(7));
    A_total = a_.at(0) * a_.at(1) * a_.at(2) * a_.at(3) * a_.at(4) * a_.at(5) * a_.at(6) * a_.at(7);

    for (int i = 0; i < A_total.rows() - 1; i++) {
      for (int j = 0; j < A_total.cols() - 1; j++) {
        R_gripper2base.at<double>(i, j) = A_total(i, j);
      }
      t_gripper2base.at<double>(i) = A_total(i, 3);
    }

    return {R_gripper2base, t_gripper2base};
  }


