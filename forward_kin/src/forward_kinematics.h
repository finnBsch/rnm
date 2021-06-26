//
// Created by rnm on 26.06.21.
//

#ifndef CATKIN_WS_FORWARD_KINEMATICS_H
#define CATKIN_WS_FORWARD_KINEMATICS_H
#include <eigen3/Eigen/Dense>
class forward_kinematics {
private:
    MatrixXd get_transformationmatrix(const float theta, const float a, const float d, const float alpha);
public:
    VectorXd get_forward_kinematics_transformation_vector(VectorXd a, VectorXd joint_angles_, VectorXd d, VectorXd alpha);
};
#endif //CATKIN_WS_FORWARD_KINEMATICS_H
