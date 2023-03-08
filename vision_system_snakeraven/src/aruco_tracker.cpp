#include "aruco_tracker.h"

/*
Arucor_tracker class:
- aruco_tracker   - constructor setting default parameters
- Get_pose        - From image, camera calibration etc. computes target pose in transform
- null_detection  - sets transform to be missing data 'identity'
*/

// Easy tutorial from http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
// //https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html
//https://docs.opencv.org/3.4/d4/d70/tutorial_hough_circle.html
//How to send arrays: http://alexsleat.co.uk/2011/07/02/ros-publishing-and-subscribing-to-arrays/#comments

//float aruco_missing[12] = {1,0,0,0,1,0,0,0,1,0,0,0}; //{n,o,a,t}
//float aruco_transform[12] = {1,0,0,0,1,0,0,0,1,0,0,0}; //{n,o,a,t}
//Mat R = (Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

aruco_tracker::aruco_tracker(){

  //Initialise Aruco Dictionary:
  dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

  //Aruco Parameters:
  marker_length = 0.05;//in metres

  //Define transform arrays initially as zeros:
  for (int i = 0; i < 12; i++){
    missing[i] = 0; transform[i] = 0;
  }
  //Ones along Diagonals
  for (int i = 0; i < 3; i++){
    missing[4 * i] = 0; transform[4 * i] = 0;
  }

  //Identity Rotation matrix:
  R = (Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
	
}

void aruco_tracker::Get_pose(const Mat image, const Mat cameraMatrix, const Mat distCoeffs, float frame){
      //Clear data
      ids.clear(); corners.clear();
      rvecs.clear(); tvecs.clear();

      //Detect new arucos
      cv::aruco::detectMarkers(image, dictionary, corners, ids);
      
      // if at least one marker detected
      if ((ids.size() > 0) && (frame > 0)) {
          cv::aruco::drawDetectedMarkers(image, corners, ids);
          cv::aruco::estimatePoseSingleMarkers(corners, marker_length, cameraMatrix, distCoeffs, rvecs, tvecs);
          //For First sighted Aruco
          cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[0], tvecs[0], 0.05);
          //Rotation matrix
          Rodrigues(rvecs[0],R);
          //Create transform vector 1x12
          for (int i=0; i<3; i++){
            for(int j=0; j<3; j++){
              transform[3*i+j] = R.at<double>(j,i);
            }
            transform[9+i] = tvecs[0][i] * 1000; //m to mm
          }
      }
      else{
        null_detection();
      }

      output_image = image;
}

void aruco_tracker::null_detection()
{
  //If not tracking
  for (int i=0; i<12; i++)
      transform[i] = missing[i]; //0; //Identity Default case when no aruco marker detected       
}