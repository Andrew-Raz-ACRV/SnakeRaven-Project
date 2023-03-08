#ifndef ARUCO_TRACKER_H_
#define ARUCO_TRACKER_H_

/*
Defines the Aruco Tracker class
Arucor_tracker class:
- aruco_tracker   - constructor setting default parameters
- Get_pose        - From image, camera calibration etc. computes target pose in transform return image
- null_detection  - sets transform to be missing data 'identity'
*/

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>

#include <opencv2/aruco.hpp> //For Aruco Marker Pose estimation
#include <opencv2/calib3d.hpp> //For Rodrigues formula

using namespace cv;
using namespace std;

class aruco_tracker
{
public:
  //Aruco Dictionary
  cv::Ptr<cv::aruco::Dictionary> dictionary;
  float marker_length;//in metres
  float missing[12];
  float transform[12];
  Mat R, output_image;

  // ids and corners for aruco
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f> > corners;
  //rotation and translation
  std::vector<cv::Vec3d> rvecs, tvecs;

	aruco_tracker();
  void Get_pose(const Mat image, const Mat cameraMatrix, const Mat distCoeffs, float frame);
  void null_detection();
	
};
#endif