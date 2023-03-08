#ifndef FEATURE_TRACKER_H_
#define FEATURE_TRACKER_H_

/*
Defines the Feature Tracker class
- establish_target_keypoints - Detects initial features inside the circle for tracking
- track_target_keypoints - Main loop code: Matches features to those initial features

Other functions:
- draw_feature_tracking - Draws the feature error and computes IBVS law called in track_target_keypoints
- drawline - draws the feature error line towards the circle
- draw_matches_cases - Puts the initial and current image side-by-side and shows the matches (not used in demo)

Andrew Razjigaev 2021
*/

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>

#include "IBVS_orientation_control.h" //IBVS class

using namespace std;
using namespace cv;

#define MIN_KEYPOINTS 3

//Feature_tracker
class Feature_tracker
{
public:
	//Image holders
	Mat greyimage, initial_image, outputImage, initial_imageLabelled, trackerimage;
	
	//Keypoint vectors and descriptor matrix
	vector<KeyPoint> keypoints, target_keypoints; 
	Mat descTarget, decKey;

	//Circle and image shape Parameters:
	Point center; 
	float height, width, radius, R, magnitude;
	
	//Status booleans
	bool established;
	
	//Thresholds:
	double nn_match_ratio;

	//Feature Tracking error:
	vector<double> deltaU, deltaV, posU, posV;

	//Initialise Visual Servo controller
	IBVS_orientation_control IBVS;

	//Initialise feature detector
  	//Ptr<AKAZE> detector = AKAZE::create(); //Good not as slow as KAZE
  	Ptr<BRISK> detector = BRISK::create(); //Good fast
  	//Ptr<KAZE> detector = KAZE::create(); //very slow CRASHES!
  	//Ptr<ORB> detector = ORB::create(); //Lots noisy and fast

  	//so called non free algorithms are patented and requires an opencv reinstall!
  	//cv::Ptr<cv::features2d::SIFT> sift = cv::features2d::SIFT::create(); ??
  	//Ptr<SIFT> detector = cv::SIFT::create(); 
  	//Ptr<SURF> detector = SURF::create();

  	//Initialise Matcher:
    Ptr<DescriptorMatcher> matcher;
    //Matcher inlier sorting variables
    vector< vector<DMatch> > nn_matches;
    vector<KeyPoint> matched1, matched2;
    vector<KeyPoint> inliers1, inliers2;
    vector<DMatch> inlier_matches;
    Mat inlier_mask, homography;
    int new_i, id;

	//Constructor
	Feature_tracker();

	//Establishes the target image features to stay inside the circle in frame 1:
	void establish_target_keypoints(const Mat image);
	//Detect features and try to match to target keypoints
	void track_target_keypoints(const Mat image, std::vector<double> V_tele);
	//Draw Feature Matches
	void draw_matches_cases(const Mat image, int state);
	//Draw feature Error
	void draw_feature_tracking(const Mat image, int state, std::vector<double> V_tele);
	void drawline(const Mat img, Point start, Point end );
	//Passive mode for inactive IBVS control:
	void passive_mode(const Mat image);

};
#endif