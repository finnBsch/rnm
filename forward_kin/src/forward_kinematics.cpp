//
// Created by rnm on 26.06.21.
//

#include "forward_kinematics.h"
#include <eigen3/Eigen/Dense>
using namespace Eigen;



class forward_kinematics{
    private:
    /*
    * This function return the 4x4 Transformationmatrix for the given Denavit Hagen Parameters
    *
    * @param: float theta, float a, float d, float alpha
    * returns: MatrixXd A
    */
        MatrixXd get_transformationmatrix(const float theta, const float a, const float d, const float alpha) {
            Matrix4d ret_mat;
            ret_mat << cos(theta), -sin(theta), 0, a,
                    sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -d * sin(alpha),
                    sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), d * cos(alpha),
                    0, 0, 0, 1;
            return ret_mat;
        }

    public:
    /*
    * This function performs the Forward kinematics and returns the overall transformation matrix to the goal position as a vector
    *
    * @param: -
    * returns: VectorXd v
    */
        VectorXd get_forward_kinematics_transformation_vector(VectorXd a, VectorXd joint_angles_, VectorXd d, VectorXd alpha) {
            std::array<MatrixXd, 8> a_;
            Matrix4d A_total;
            VectorXd vector;

            for(int i = 0; i<7; i++){
                a_.at(i) = get_transformationmatrix(joint_angles_(i), a(i), d(i), alpha(i));
            }
            a_.at(7) = get_transformationmatrix(0, a(7), d(7), alpha(7));
            A_total = a_.at(0) * a_.at(1) * a_.at(2) * a_.at(3) * a_.at(4) * a_.at(5) * a_.at(6) * a_.at(7);

            vector<< A_total(0,0), A_total(1,0), A_total(2,0),
                     A_total(0,1), A_total(1,1), A_total(2,1),
                     A_total(0,2), A_total(1,2), A_total(2,2),
                     A_total(0,3), A_total(1,3), A_total(2,3);

            return vector;
    }
};

