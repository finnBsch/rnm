/*
 * This is a file to add any convert type functions
 * author: Sean Maroofi
 * added: 29.5.21
 * last edited: 29.5.21
 */

#include <eigen3/Eigen/Dense>


using namespace Eigen;


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