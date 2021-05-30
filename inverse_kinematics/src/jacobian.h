//
// Created by tolga on 30.05.21.
//

#ifndef SRC_JACOBIAN_H
#define SRC_JACOBIAN_H

#endif //SRC_JACOBIAN_H

MatrixXd calculateJacobian( sensor_msgs::JointState& msg){
    theta1= msg.position[0];
    theta2= msg.position[1];
    theta3= msg.position[2];
    theta4= msg.position[3];
    theta5= msg.position[4];
    theta6= msg.position[5];
    theta7= msg.position[6];
    MatrixXd jacobian(12,7);

    jacobian <<




    return jacobian;
}
