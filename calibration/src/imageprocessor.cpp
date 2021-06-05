
#include "imageprocessor.h"

using namespace std;
using namespace cv;

//Perform the stereoCalibration

irFrameSize;
rgbFrameSize;
vector<vector<Point3f> > object_points;
vector<vector<Point2f> > imagePoints1, imagePoints2;
vector<vector<Point2f> > left_img_points, right_img_points;
vector<vector<Point2f> > rgbcorners;
vector<vector<Point2f> > ircorners;



  std::vector<std::vector<cv::Point3f >> irObjP;// Checkerboard world coordinates
  std::vector<std::vector<cv::Point3f >> rgbObjP;// Checkerboard world coordinates

  // Define the world coordinates for 3D points

  std::vector<cv::Point3f> rgbobjp; //define the object point in 3D
  for (int i = 1; i < checkerBoard[1]; i++) {
    for (int j = 1; j < checkerBoard[0]; j++) {
      rgbobjp.push_back(cv::Point3f(j * fieldSize, i * fieldSize, 0));
    }
  }
  // Define the world coordinates for 3D points of the ir frame
  std::vector<cv::Point3f> irobjp; //define the object point in 3D
  for (int i = 1; i < checkerBoard[1]; i++) {
    for (int j = 1; j < checkerBoard[0]; j++) {
      irobjp.push_back(cv::Point3f(j * fieldSize, i * fieldSize, 0));
    }
  }

  cout << "All IR Corners detected and safed in ircorners\n";

//Calibrate the rgb frame intrinsics
  cv::Mat rgbK;  // calibration Matrix K
  Mat rgbk = (Mat1d(1, 8));
  std::vector<cv::Mat> rvecs, tvecs;
  std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
  int flags =
      cv::CALIB_FIX_S1_S2_S3_S4
      + cv::CALIB_FIX_TAUX_TAUY
      + cv::CALIB_RATIONAL_MODEL; //used to compute k4-k6

  std::cout << "Calibrating rgb intrinsics...\n" << std::endl;

  float rgberror = cv::calibrateCamera(rgbObjP, rgbcorners, rgbFrameSize, rgbK, rgbk, rvecs, tvecs, flags);
  std::cout << "Reprojection error of rgb frames = " << rgberror << "\nK =\n"
            << rgbK << "\nk=\n"
            << rgbk << std::endl;
  //Calibrate the ir frame intrinsics
  cv::Mat irK;  // calibration Matrix K
  cv::Mat irk = (Mat1d(1, 8));
  std::vector<cv::Mat> irrvecs, irtvecs;
  std::vector<double> irIntrinsics, irExtrinsics, irperViewErrors;
  int irflags =
      cv::CALIB_FIX_S1_S2_S3_S4
      + cv::CALIB_FIX_TAUX_TAUY
      + cv::CALIB_RATIONAL_MODEL; //used to compute k4-k6
  std::cout << "Calibrating ir intrinsics...\n" << std::endl;
  // 4. Call "float error = cv::calibrateCamera()" with the input coordinates
  // and output parameters as declared above...
  float irerror = cv::calibrateCamera(irObjP, ircorners, irFrameSize, irK, irk, irrvecs, irtvecs, irflags);
  std::cout << "Reprojection error of the ir frames = " << irerror << "\nK =\n"
            << irK << "\nk=\n"
            << irk << std::endl;

  ///////////////////////////////////////////////////////
    vector<Point3f> obj;
    for (int j = 1; j < checkerBoard[1]; j++){
      for (int k = 1; k < checkerBoard[0]; k++) {
        obj.push_back(Point3f((float) k * fieldSize, (float) j * fieldSize, 0));
      }
    }

  std::cout << "Starting Calibration\n";
  cv::Mat K1, K2, R, F, E;
  cv::Vec3d T;
  cv::Mat D1, D2;
  K1 = rgbK;
  K2 = irK;
  D1 = rgbk;
  D2 = irk;


  int stereoflags =
      CALIB_FIX_INTRINSIC
      + CALIB_FIX_S1_S2_S3_S4
      + CALIB_FIX_TAUX_TAUY
      + CALIB_RATIONAL_MODEL;


  std::cout << "Compute stereocalibration...\n";
  double rms = stereoCalibrate(object_points, left_img_points, right_img_points, K1, D1, K2, D2, rgbFrameSize,
                               R, T, E, F, stereoflags);
  std::cout << "Done with RMS error=" << rms << endl;


// Precompute lens correction interpolation
  std::cout << "Done Stereocalibration\n";


  //Rectification
  std::cout <<"Starting Rectification\n";
  cv::Mat R1, R2, P1, P2, Q;
  stereoRectify(K1, D1, K2, D2, irFrameSize, R, T, R1, R2, P1, P2, Q);
  printf("Done Rectification\n");


  //Save as YML
  cv::FileStorage fs1("CalibrationParam.yml", cv::FileStorage::WRITE);
  fs1 << "K1" << K1;
  fs1 << "K2" << K2;
  fs1 << "D1" << D1;
  fs1 << "D2" << D2;
  fs1 << "R" << R;
  fs1 << "T" << T;
  fs1 << "E" << E;
  fs1 << "F" << F;
  fs1 << "R1" << R1;
  fs1 << "R2" << R2;
  fs1 << "P1" << P1;
  fs1 << "P2" << P2;
  fs1 << "Q" << Q;


// Show lens corrected ir images
  //  for (auto const &f : irFileNames) {
  //     std::cout << std::string(f) << std::endl;
  //     cv::Mat irimg = cv::imread(f, cv::IMREAD_COLOR);
  //     cv::Mat irimgUndistorted;
// 5. Remap the image using the precomputed interpolation maps.
  //      cv::remap(irimg, irimgUndistorted, irmapX, irmapY, cv::INTER_LINEAR);
//Display the undistorted images
  // cv::imshow("undistorted image", irimgUndistorted);
  // cv::waitKey(0);
  //  }



  cout << "Calibration successful" << std::endl;
  return 0;

































void imageprocessor::cornerDetection(cv::Mat images, std::vector<std::vector<Point2f>>& corners){
  ROS_INFO("Detecting corners...");
  images = cv_bridge::toCvShare(msg, "bgr8")->img;
cv::Mat gray;
cv::cvtColor(images, gray, cv::COLOR_BGR2GRAY);


bool rgbpatternfound = findChessboardCorners(images, patternSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
cv::drawChessboardCorners(images, patternSize, corners, rgbpatternfound);
bool irpatternfound = findChessboardCorners(images, patternSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
cv::drawChessboardCorners(images, patternSize, corners, rgbpatternfound);

if (rgbpatternfound && irpatternfound) {
  cv::cornerSubPix(gray, corners,cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 50, 0.001));
  collected_frame.push_back(make_tuple());

  cv::cornerSubPix(gray, corners,cv::Size(20,20), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.001));
  collected_frame.push_back(make_tuple());
  ROS_INFO( "of" imgsum "Images")
}
else{
  collected_frame.erase(next(collected_frame.begin(), )) //globaler Zähler der gespeicherten images muss deklariert werden
  ROS_INFO("Checkerboard not detected\n",  "of " imgsum "Images");
          cout << "\n!!!!!!!Checkerboard not detected!!!!!!!\n";

}
}









void imageprocessor::cameraCalibration(){ //Calibration Camera Intrinsics
  ROS_INFO("Beginn Calibration Process...")
  std::vector<std::vector<cv::Point2f>> imagePoints;
  std::vector<std::vector<Point3f>> objectPoints(1);
  loadFromFolder();
  cornerDetection(images, imagePoints);
  objectPoints.resize(imagePoints.size(), objectPoints[0]);

  // Define board position in 3D space
  std::vector<cv::Point3f> corners; //define the object point in 3D
  for (int i = 1; i < checkerBoard[1]; i++) {
    for (int j = 1; j < checkerBoard[0]; j++) {
      corners.push_back(cv::Point3f(j * fieldsize, i * fieldsize, 0.0f));
    }
  }

  cv::Mat cameraMatrix(cv::Matx33f::eye());  // calibration Matrix K
  Mat distCoeffs = (Mat1d(1, 8)); // Distortion coeffizients
  std::vector<cv::Mat> rvecs, tvecs;
  std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
  int rgbflags =
      cv::CALIB_FIX_ASPECT_RATIO
      + cv::CALIB_FIX_S1_S2_S3_S4
      + cv::CALIB_FIX_TAUX_TAUY
      + cv::CALIB_RATIONAL_MODEL;
  cv::Size frameSize(2048, 1536);
  std::cout << "Calibrating rgb intrinsics...\n";
  // Call "float error = cv::calibrateCamera()" with the input coordinates
  // and output parameters as declared above...
  float error = cv::calibrateCamera(objectPoints, imagePoints, frameSize, cameraMatrix, distCoeffs,
                                       rvecs, tvecs, rgbflags);
  std::cout << "Reprojection error of rgb frames = " << error << "\nK =\n"
            << cameraMatrix << "\nk=\n"
            << distCoeffs << std::endl;
}

void imageprocessor::test_ir(const sensor_msgs::ImageConstPtr& msg){
  cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
  // check image, wenn gut, dann in den vector pushen
  std::vector<std::vector<cv::Point2f>> corners(img.size());

  cv::Mat gray;
  cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  bool irpatternfound = findChessboardCorners(
      img, patternSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
  cv::drawChessboardCorners(img, patternSize, corners, irpatternfound);

  if (irpatternfound) {
    cv::cornerSubPix(
        gray, corners, cv::Size(20, 20), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.0001));
    collected_frame.push_back(make_tuple())
  } else {
    collected_frame.erase(
        next(collected_frame.begin(), i))  // globaler Zähler der gespeicherten images muss deklariert werden
        cout
        << "\n!!!!!!!Checkerboard not detected!!!!!!!\n"
  }
}
