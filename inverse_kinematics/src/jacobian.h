//
// Created by tolga on 30.05.21.
//

#ifndef SRC_JACOBIAN_H
#define SRC_JACOBIAN_H
#include <eigen3/Eigen/Dense>
#include "sensor_msgs/JointState.h"
#endif //SRC_JACOBIAN_H
using namespace Eigen;
MatrixXd calculateJacobian(const VectorXd jointAngles);