//
// Created by lars on 21.06.21.
//
#include "cv_handeye.h"
Mat homogeneousInverse(const Mat& T)
{
  CV_Assert(T.rows == 4 && T.cols == 4);

  Mat R = T(Rect(0, 0, 3, 3));
  Mat t = T(Rect(3, 0, 1, 3));
  Mat Rt = R.t();
  Mat tinv = -Rt * t;
  Mat Tinv = Mat::eye(4, 4, T.type());
  Rt.copyTo(Tinv(Rect(0, 0, 3, 3)));
  tinv.copyTo(Tinv(Rect(3, 0, 1, 3)));

  return Tinv;
}
void calibrateHandEye(InputArrayOfArrays R_gripper2base, InputArrayOfArrays t_gripper2base,
                      InputArrayOfArrays R_target2cam, InputArrayOfArrays t_target2cam,
                      OutputArray R_cam2gripper, OutputArray t_cam2gripper)
{
  CV_Assert(R_gripper2base.isMatVector() && t_gripper2base.isMatVector() &&
            R_target2cam.isMatVector() && t_target2cam.isMatVector());

  std::vector<Mat> R_gripper2base_, t_gripper2base_;
  R_gripper2base.getMatVector(R_gripper2base_);
  t_gripper2base.getMatVector(t_gripper2base_);

  std::vector<Mat> R_target2cam_, t_target2cam_;
  R_target2cam.getMatVector(R_target2cam_);
  t_target2cam.getMatVector(t_target2cam_);

  CV_Assert(R_gripper2base_.size() == t_gripper2base_.size() &&
            R_target2cam_.size() == t_target2cam_.size() &&
            R_gripper2base_.size() == R_target2cam_.size());
  CV_Assert(R_gripper2base_.size() >= 3);

  //Notation used in Tsai paper
  //Defines coordinate transformation from G (gripper) to RW (robot base)
  std::vector<Mat> Hg;
  Hg.reserve(R_gripper2base_.size());
  for (size_t i = 0; i < R_gripper2base_.size(); i++)
  {
    Mat m = Mat::eye(4, 4, CV_64FC1);
    Mat R = m(Rect(0, 0, 3, 3));
    R_gripper2base_[i].convertTo(R, CV_64F);

    Mat t = m(Rect(3, 0, 1, 3));
    t_gripper2base_[i].convertTo(t, CV_64F);

    Hg.push_back(m);
  }

  //Defines coordinate transformation from CW (calibration target) to C (camera)
  std::vector<Mat> Hc;
  Hc.reserve(R_target2cam_.size());
  for (size_t i = 0; i < R_target2cam_.size(); i++)
  {
    Mat m = Mat::eye(4, 4, CV_64FC1);
    Mat R = m(Rect(0, 0, 3, 3));
    R_target2cam_[i].convertTo(R, CV_64F);

    Mat t = m(Rect(3, 0, 1, 3));
    t_target2cam_[i].convertTo(t, CV_64F);

    Hc.push_back(m);
  }

  Mat Rcg = Mat::eye(3, 3, CV_64FC1);
  Mat Tcg = Mat::zeros(3, 1, CV_64FC1);


  //can switch to Tsai here
  calibrateHandEyeTsai(Hg, Hc, Rcg, Tcg);

  Rcg.copyTo(R_cam2gripper);
  Tcg.copyTo(t_cam2gripper);
}

Mat skew(const Mat& v)
{
  CV_Assert(v.type() == CV_64FC1 && v.rows == 3 && v.cols == 1);

  double vx = v.at<double>(0,0);
  double vy = v.at<double>(1,0);
  double vz = v.at<double>(2,0);
  return (Mat_<double>(3,3) << 0, -vz, vy,
      vz, 0, -vx,
      -vy, vx, 0);
}

// R = quatMinimal2rot(q)
//
// q - 3x1 unit quaternion <qx, qy, qz>
// R - 3x3 rotation matrix
// q = sin(theta/2) * v
// theta - rotation angle
// v     - unit rotation axis, |v| = 1
Mat quatMinimal2rot(const Mat& q)
{
  CV_Assert(q.type() == CV_64FC1 && q.rows == 3 && q.cols == 1);

  Mat p = q.t()*q;
  double w = sqrt(1 - p.at<double>(0,0));

  Mat diag_p = Mat::eye(3,3,CV_64FC1)*p.at<double>(0,0);
  return 2*q*q.t() + 2*w*skew(q) + Mat::eye(3,3,CV_64FC1) - 2*diag_p;
}

// q = rot2quat(R)
//
// q - 4x1 unit quaternion <qw, qx, qy, qz>
// R - 3x3 rotation matrix
// Reference: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
Mat rot2quat(const Mat& R)
{
  CV_Assert(R.type() == CV_64FC1 && R.rows >= 3 && R.cols >= 3);

  double m00 = R.at<double>(0,0), m01 = R.at<double>(0,1), m02 = R.at<double>(0,2);
  double m10 = R.at<double>(1,0), m11 = R.at<double>(1,1), m12 = R.at<double>(1,2);
  double m20 = R.at<double>(2,0), m21 = R.at<double>(2,1), m22 = R.at<double>(2,2);
  double trace = m00 + m11 + m22;

  double qw, qx, qy, qz;
  if (trace > 0) {
    double S = sqrt(trace + 1.0) * 2; // S=4*qw
    qw = 0.25 * S;
    qx = (m21 - m12) / S;
    qy = (m02 - m20) / S;
    qz = (m10 - m01) / S;
  } else if (m00 > m11 && m00 > m22) {
    double S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx
    qw = (m21 - m12) / S;
    qx = 0.25 * S;
    qy = (m01 + m10) / S;
    qz = (m02 + m20) / S;
  } else if (m11 > m22) {
    double S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
    qw = (m02 - m20) / S;
    qx = (m01 + m10) / S;
    qy = 0.25 * S;
    qz = (m12 + m21) / S;
  } else {
    double S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
    qw = (m10 - m01) / S;
    qx = (m02 + m20) / S;
    qy = (m12 + m21) / S;
    qz = 0.25 * S;
  }

  return (Mat_<double>(4,1) << qw, qx, qy, qz);
}
void  calibrateHandEyeTsai(const std::vector<Mat>& Hg, const std::vector<Mat>& Hc,
                           Mat& R_cam2gripper, Mat& t_cam2gripper)
{
  //Number of unique camera position pairs
  int K = static_cast<int>((Hg.size()*Hg.size() - Hg.size()) / 2.0);
  //Will store: skew(Pgij+Pcij)
  Mat A(3*K, 3, CV_64FC1);
  //Will store: Pcij - Pgij
  Mat B(3*K, 1, CV_64FC1);

  std::vector<Mat> vec_Hgij, vec_Hcij;
  vec_Hgij.reserve(static_cast<size_t>(K));
  vec_Hcij.reserve(static_cast<size_t>(K));

  int idx = 0;
  for (size_t i = 0; i < Hg.size(); i++)
  {
    for (size_t j = i+1; j < Hg.size(); j++, idx++)
    {
      //Defines coordinate transformation from Gi to Gj
      //Hgi is from Gi (gripper) to RW (robot base)
      //Hgj is from Gj (gripper) to RW (robot base)
      Mat Hgij = homogeneousInverse(Hg[j]) * Hg[i]; //eq 6
      vec_Hgij.push_back(Hgij);
      //Rotation axis for Rgij which is the 3D rotation from gripper coordinate frame Gi to Gj
      Mat Pgij = 2*rot2quatMinimal(Hgij);

      //Defines coordinate transformation from Ci to Cj
      //Hci is from CW (calibration target) to Ci (camera)
      //Hcj is from CW (calibration target) to Cj (camera)
      Mat Hcij = Hc[j] * homogeneousInverse(Hc[i]); //eq 7
      vec_Hcij.push_back(Hcij);
      //Rotation axis for Rcij
      Mat Pcij = 2*rot2quatMinimal(Hcij);

      //Left-hand side: skew(Pgij+Pcij)
      skew(Pgij+Pcij).copyTo(A(Rect(0, idx*3, 3, 3)));
      //Right-hand side: Pcij - Pgij
      Mat diff = Pcij - Pgij;
      diff.copyTo(B(Rect(0, idx*3, 1, 3)));
    }
  }

  Mat Pcg_;
  //Rotation from camera to gripper is obtained from the set of equations:
  //    skew(Pgij+Pcij) * Pcg_ = Pcij - Pgij    (eq 12)
  solve(A, B, Pcg_, DECOMP_SVD);

  Mat Pcg_norm = Pcg_.t() * Pcg_;
  //Obtained non-unit quaternion is scaled back to unit value that
  //designates camera-gripper rotation
  Mat Pcg = 2 * Pcg_ / sqrt(1 + Pcg_norm.at<double>(0,0)); //eq 14

  Mat Rcg = quatMinimal2rot(Pcg/2.0);

  idx = 0;
  for (size_t i = 0; i < Hg.size(); i++)
  {
    for (size_t j = i+1; j < Hg.size(); j++, idx++)
    {
      //Defines coordinate transformation from Gi to Gj
      //Hgi is from Gi (gripper) to RW (robot base)
      //Hgj is from Gj (gripper) to RW (robot base)
      Mat Hgij = vec_Hgij[static_cast<size_t>(idx)];
      //Defines coordinate transformation from Ci to Cj
      //Hci is from CW (calibration target) to Ci (camera)
      //Hcj is from CW (calibration target) to Cj (camera)
      Mat Hcij = vec_Hcij[static_cast<size_t>(idx)];

      //Left-hand side: (Rgij - I)
      Mat diff = Hgij(Rect(0,0,3,3)) - Mat::eye(3,3,CV_64FC1);
      diff.copyTo(A(Rect(0, idx*3, 3, 3)));

      //Right-hand side: Rcg*Tcij - Tgij
      diff = Rcg*Hcij(Rect(3, 0, 1, 3)) - Hgij(Rect(3, 0, 1, 3));
      diff.copyTo(B(Rect(0, idx*3, 1, 3)));
    }
  }

  Mat Tcg;
  //Translation from camera to gripper is obtained from the set of equations:
  //    (Rgij - I) * Tcg = Rcg*Tcij - Tgij    (eq 15)
  solve(A, B, Tcg, DECOMP_SVD);

  R_cam2gripper = Rcg;
  t_cam2gripper = Tcg;
}
Mat rot2quatMinimal(const Mat& R)
{
  CV_Assert(R.type() == CV_64FC1 && R.rows >= 3 && R.cols >= 3);

  double m00 = R.at<double>(0,0), m01 = R.at<double>(0,1), m02 = R.at<double>(0,2);
  double m10 = R.at<double>(1,0), m11 = R.at<double>(1,1), m12 = R.at<double>(1,2);
  double m20 = R.at<double>(2,0), m21 = R.at<double>(2,1), m22 = R.at<double>(2,2);
  double trace = m00 + m11 + m22;

  double qx, qy, qz;
  if (trace > 0) {
    double S = sqrt(trace + 1.0) * 2; // S=4*qw
    qx = (m21 - m12) / S;
    qy = (m02 - m20) / S;
    qz = (m10 - m01) / S;
  } else if ((m00 > m11)&(m00 > m22)) {
    double S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx
    qx = 0.25 * S;
    qy = (m01 + m10) / S;
    qz = (m02 + m20) / S;
  } else if (m11 > m22) {
    double S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
    qx = (m01 + m10) / S;
    qy = 0.25 * S;
    qz = (m12 + m21) / S;
  } else {
    double S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
    qx = (m02 + m20) / S;
    qy = (m12 + m21) / S;
    qz = 0.25 * S;
  }

  return (Mat_<double>(3,1) << qx, qy, qz);
}