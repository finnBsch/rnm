
#include <iostream>
#include <stdlib.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <stdio.h>

using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    (void) argc;
    (void) argv;

    cv::Size rgbFrameSize(2048, 1536);
    cv::Size irFrameSize(640, 576);

    std::vector<cv::String> rgbFileNames;
    std::vector<cv::String> irFileNames;
    //std::string rgbFolder("/home/lars/CLionProjects/CameraCalibrationtests/cal_imgs/rgb/*.jpg");
    std::string rgbFolder("/home/nico/catkin_ws/src/frame_reader/cal_imgs/rgb/*.jpg");
    cv::glob(rgbFolder, rgbFileNames, true); // load rgb images into opencv
    std::string irFolder("/home/nico/catkin_ws/src/frame_reader/cal_imgs/ir/*.jpg");
    //std::string irFolder("/home/lars/CLionProjects/CameraCalibrationtests/cal_imgs/ir/*.jpg");
    cv::glob(irFolder, irFileNames, true);

    std::vector<std::vector<cv::Point3f >> irObjP;// Checkerboard world coordinates
    std::vector<std::vector<cv::Point3f >> rgbObjP;// Checkerboard world coordinates


    // Define checkerboard Parameters for both frames
    int fieldSize = 40;
    int checkerBoard[2] = {9, 6}; // Checkerboard pattern
    cv::Size patternSize(9 - 1, 6 - 1); // Number of inner corners per a chessboard row and column


    std::vector<std::vector<cv::Point2f>> rgbcorners(rgbFileNames.size()); // corners in rgb frames
    std::vector<std::vector<cv::Point2f>> ircorners(irFileNames.size()); // corners in ir frames
    std::vector<cv::Point2f> rgbimgp; //Define the rgb img point
    std::vector<cv::Point2f> irimgp; //Define the ir img point


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


    //Detect corners of the checkerboard in the RGB Frames
    cv::Mat rgbimg;
    std::size_t i2 = 0;
    std::size_t i = 0;
    std::size_t i_deleted = 0;
    for (auto const &f : rgbFileNames){
        std::cout << std::string(f) << std::endl;
        rgbimg = cv::imread(f); //Load the images
        cv::Mat rgbgray;//grayscale the image
        cv::cvtColor(rgbimg, rgbgray, cv::COLOR_RGB2GRAY);
        bool rgbPatternFound = cv::findChessboardCorners(rgbgray, patternSize, rgbcorners[i2],

                                                         cv::CALIB_CB_ADAPTIVE_THRESH
                                                         //+ cv::CALIB_CB_NORMALIZE_IMAGE
                                                         + cv::CALIB_CB_FILTER_QUADS
        );
        // Use cv::cornerSubPix() to refine the found corner detections with default values given by opencv
        if (rgbPatternFound) {
            cv::cornerSubPix(rgbgray, rgbcorners[i2], cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.01));
            rgbObjP.push_back(rgbobjp);
            i2++;
        }
        else {
            rgbcorners.erase(next(rgbcorners.begin(), i-i_deleted));
            cout << endl << "!!!!!!!!File " << rgbFileNames[i] << "not used!!!!!!!!" << endl << endl;
            i_deleted++;
        }


        i++;
    }
    cout << "All RGB Corners detected and safed in rgbcorners\n";



//for(int i =0; i <  rgbFileNames.size() ; i++){
//  // Display the detected pattern on the chessboard
//  rgbimg = ;
//  cv::drawChessboardCorners(rgbimg, patternSize, rgbcorners[i], true);
//  cv::imshow("RGB chessboard corner detection", rgbimg);
//  cv::waitKey(0);
//}


    //Detect corners of the checkerboard in the IR Frames
    cv::Mat irimg;
    std::size_t m = 0;
    std::size_t m2 = 0 ;
    std::size_t m_deleted = 0;
    for (auto const &f : irFileNames) {
        std::cout << std::string(f) << std::endl;
        irimg = cv::imread(f); //Load the images
        cv::Mat irgray;//grayscale the image
        cv::cvtColor(irimg, irgray, cv::COLOR_RGB2GRAY);
        bool irpatternFound = cv::findChessboardCorners(irgray, patternSize, ircorners[m2],
                                                        cv::CALIB_CB_ADAPTIVE_THRESH
                                                        + cv::CALIB_CB_NORMALIZE_IMAGE);

        // Display the detected chessboard pattern on the ir frames
        //cv::drawChessboardCorners(irimg, patternSize, ircorners[m], irpatternFound);
        //cv::imshow("IR chessboard corner detection", irimg);
        //cv::waitKey(1);
        if (irpatternFound) {
            cv::cornerSubPix(irgray, ircorners[m2], cv::Size(20, 20), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.0001));
            irObjP.push_back(irobjp);
            m2++;
        }
        else {
            ircorners.erase(next(ircorners.begin(), m-m_deleted));
            cout << endl << "!!!!!!!!File " << irFileNames[m] << "not used!!!!!!!!" << endl << endl;
            m_deleted++;
        }
        m++;
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

    std::cout << "Calibrating rgb intrinsics...\n" << endl;


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

    cv::Mat rgbmapX, rgbmapY;
    cv::initUndistortRectifyMap(rgbK, rgbk, cv::Matx33f::eye(), rgbK, rgbFrameSize, CV_32FC1,
                                rgbmapX, rgbmapY);
    cv::Mat irmapX, irmapY;
    cv::initUndistortRectifyMap(irK, irk, cv::Matx33f::eye(), irK, irFrameSize, CV_32FC1,
                                irmapX, irmapY);


    // Show lens corrected rgb images
    for (auto const &f : rgbFileNames) {
        std::cout << std::string(f) << std::endl;

        cv::Mat rgbimg = cv::imread(f, cv::IMREAD_COLOR);
        cv::Mat rgbimgUndistorted;
// 5. Remap the image using the precomputed interpolation maps.
        cv::remap(rgbimg, rgbimgUndistorted, rgbmapX, rgbmapY, cv::INTER_LINEAR);

//Display the undistorted images
        //  cv::imshow("undistorted image", rgbimgUndistorted);
        // cv::waitKey(0);
    }

    ///////////////////////////////////////////////////////
    //Perform the stereoCalibration
    vector<vector<Point3f> > object_points;
    vector<vector<Point2f> > imagePoints1, imagePoints2;
    vector<Point2f> corners1, corners2;
    vector<vector<Point2f> > left_img_points, right_img_points;
    Mat img1, img2;
    //Size board_size = Size(board_width, board_height);
    // int board_n = board_width * board_height;
    int num_deleted = 0;
    int t =0;
    for (int i = 0; i < rgbFileNames.size(); i++) {


        img1 = cv::imread(rgbFileNames[i]); //Load the images
        img2 = cv::imread(irFileNames[i]); //Load the images
        cv::Mat gray1;//grayscale the rgb image
        cv::cvtColor(img1, gray1, cv::COLOR_BGR2GRAY);
        cv::Mat gray2;//grayscale the ir image
        cv::cvtColor(img2, gray2, cv::COLOR_BGR2GRAY);

        bool found1 = cv::findChessboardCorners(gray1, patternSize, corners1,
                                                CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_FILTER_QUADS);
//CALIB_CB_NORMALIZE_IMAGE);

        bool found2 = cv::findChessboardCorners(gray2, patternSize, corners2,
                                                CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);

        vector<Point3f> obj;
        for (int j = 1; j < checkerBoard[1]; j++){
            for (int k = 1; k < checkerBoard[0]; k++) {
                obj.push_back(Point3f((float) k * fieldSize, (float) j * fieldSize, 0));
            }
        }
        if (found1 && found2) {
            //Refine CornerDetection
            cv::cornerSubPix(gray1, corners1, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.01));

            cv::drawChessboardCorners(gray1, patternSize, corners1, found1);

            cv::cornerSubPix(gray2, corners2, cv::Size(20, 20), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.1));

            cv::drawChessboardCorners(gray2, patternSize, corners2, found2);
            cout << t << "good framepairs" << endl;
            t++;
            imagePoints1.push_back(corners1);
            imagePoints2.push_back(corners2);
            object_points.push_back(obj);
            cout << "leftImg: " << rgbFileNames[i] << " and rightImg: " << irFileNames[i] << endl;
        }
        else {
            corners1.erase(next(corners1.begin(), i-num_deleted));
            corners2.erase(next(corners2.begin(), i-num_deleted));
            num_deleted++;

            cout << endl << "!!!!!!!!Pairs " << irFileNames[i] << "not used!!!!!!!!" << endl << endl;
        }
    }

    for (int i = 0; i < imagePoints1.size(); i++) {
        vector<Point2f> v1, v2;
        for (int j = 0; j < imagePoints1[i].size(); j++) {
            v1.push_back(Point2f((double) imagePoints1[i][j].x, (double) imagePoints1[i][j].y));
            v2.push_back(Point2f((double) imagePoints2[i][j].x, (double) imagePoints2[i][j].y));
        }
        left_img_points.push_back(imagePoints1[i]);
        right_img_points.push_back(imagePoints2[i]);
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
    cout << "Done with RMS error=" << rms << endl;


// Precompute lens correction interpolation
    cout << "Done Stereocalibration\n";


    //Rectification
    cout <<"Starting Rectification\n";
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

//TODO fileformat used for the hand-eye calibration



//HandEyeCalibration: notation based on Tsai-Lenz:
//G(i)=>E(i): The endeffector coordinate system. That is, the coordinate frame fixed on the robot endeffector and as the robot moves, it moves with the gripper
//C(i): The camera coordinate system. That is, the coordinate frame fixed on the camera, with the z-axis cooinciding with the optical axis, and the x,y axes parallel to the image X,Y axes
//CW: The calibration block world coordinate frame. This is an arbitrary selected coordinate frame set on the calibration block so that the coordinate of each target point on the calibration block is known a priori relatie to VW
//RW: The robot world coordinate frame.It is fixed in the robot work station, and as the robot arm moves around, the encoder output of all the robot joints enables the system to tell where the gripper is relative to RW

//cv::Mat X(4, 4, CV_64FC1); //Transformation from endeffector E to Camera UNKNOWN
//cv::Mat Z(4, 4, CV_64FC1); //Transformation from Robot RW to World CW UNKNOWN

//TODO generate the right input for the calibrateHandEye function

}
