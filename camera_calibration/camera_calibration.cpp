#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char **argv) {

  (void)argc;
  (void)argv;

  std::vector<cv::String> fileNames;
  cv::glob("/home/nico/catkin_ws/src/frame_reader/cal_imgs/bgr/pose_*.jpg", fileNames, false);
  cv::Size patternSize(9 - 1, 6 - 1);
  std::vector<std::vector<cv::Point2f>> q(fileNames.size());

  std::vector<std::vector<cv::Point3f>> Q;
  // 1. Generate checkerboard (world) coordinates Q. The board has 9x6 squares
  // fields with a size of 44mm

  int checkerBoard[2] = {9,6};
  int fieldSize = 44;
  // Defining the world coordinates for 3D points
  std::vector<cv::Point3f> objp;
  for(int i = 1; i<checkerBoard[1]; i++){
    for(int j = 1; j<checkerBoard[0]; j++){
      objp.push_back(cv::Point3f(j*fieldSize,i*fieldSize,0));
    }
  }

  std::vector<cv::Point2f> imgPoint;
  // Detect feature points on checkerboard
  std::size_t i = 0;
  for (auto const &f : fileNames) {
    std::cout << std::string(f) << std::endl;

    // 2. Read images
    cv::Mat img = cv::imread(fileNames[i]);
    cv::Mat gray;

    cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);

    // if all points found: grayscale + pattern size + cv stuff --> camera coordinates q
    bool patternFound = cv::findChessboardCorners(gray, patternSize, q[i], cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

    // 2. Use cv::cornerSubPix() to refine the found corner detections
    // if found: push to Q
    if(patternFound){
      cv::cornerSubPix(gray, q[i],cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
      Q.push_back(objp);
    }

    // Display
    cv::drawChessboardCorners(img, patternSize, q[i], patternFound);
    cv::imshow("chessboard detection", img);
    cv::waitKey(1);

    i++;
  }


  // outputs from calibrateCamera
  cv::Matx33f K(cv::Matx33f::eye());  // intrinsic camera matrix
  cv::Vec<float, 5> k(0, 0, 0, 0, 0); // distortion coefficients
  //cv::Vec<float, 8> k(0, 0, 0, 0, 0,0,0,0); // distortion coefficients

  // rotation + translation vector
  std::vector<cv::Mat> rvecs, tvecs;
  std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
  // opencv stuff
  int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 +
              cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;
  cv::Size frameSize(2048, 1536);//

  std::cout << "Calibrating..." << std::endl;
  // 4. Call "float error = cv::calibrateCamera()" with the input coordinates
  // and output parameters as declared above...

  float error = cv::calibrateCamera(Q, q, frameSize, K, k, rvecs, tvecs, flags);

  std::cout << "Reprojection error = " << error << "\nK =\n"
            << K << "\nk=\n"
            << k << "\nrvecs=\n" << std::endl;


  // Precompute lens correction interpolation
  cv::Mat mapX, mapY;
  cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1,
                              mapX, mapY);

  // Show lens corrected images
  for (auto const &f : fileNames) {
    std::cout << std::string(f) << std::endl;

    cv::Mat img = cv::imread(f, cv::IMREAD_COLOR);

    cv::Mat imgUndistorted;
    // 5. Remap the image using the precomputed interpolation maps.
    cv::remap(img, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);

    // Display
    cv::imshow("undistorted image", imgUndistorted);
    cv::waitKey(0);
  }

  return 0;
}