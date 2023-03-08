#include "Feature_tracker.h"

/*
Defines the Feature Tracker class
- establish_target_keypoints - Detects initial features inside the circle for tracking
- track_target_keypoints - Main loop code: Matches features to those initial features

Other functions:
- draw_feature_tracking - Draws the feature error and computes IBVS law called in track_target_keypoints
- drawline - draws the feature error line towards the circle
- draw_matches_cases - Puts the initial and current image side-by-side and shows the matches (not used in demo)

Andrew Razjigaev 2021

Useful source: 
https://docs.opencv.org/3.4/db/d70/tutorial_akaze_matching.html
*/

//Constructor
Feature_tracker::Feature_tracker()
{
    //Initialise matcher:
    matcher = DescriptorMatcher::create("BruteForce-Hamming");
	//Initially not established
	established = false;
	//Set Thresholds upon initialisation
	nn_match_ratio = 0.8;
}

/*
Shows the view of the tracker but does do IBVS:
*/
void Feature_tracker::passive_mode(const Mat image)
{
    //Tracking is not established:
    established = false;
    //Define circle parameters:
    height = image.cols; 
    width = image.rows;
    radius = height/4;
    center = Point(height/2, width/2); 
    //Convert to Grayscale before feature detection
    cvtColor(image, greyimage, CV_BGR2GRAY);

    //Clear data and show current visible Keypoints:
    keypoints.clear();
    detector->detectAndCompute(greyimage, noArray(), keypoints, decKey);    
    //Draw keypoints tracker image: trackerimage
    drawKeypoints(image, keypoints, trackerimage, Scalar(255,255,255) ,cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    circle(trackerimage, center, radius, CV_RGB(255,0,0));
    drawMarker(trackerimage, center,  Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);
    IBVS.Null_Control();
    outputImage = trackerimage;
}

/*
Draws the feature error vector from the image and compute the IBVS control law
*/
void Feature_tracker::draw_feature_tracking(const Mat image, int state, std::vector<double> Vteleop)
{
    //Prepare Feature error vector:
    deltaU.clear();
    deltaV.clear();
    posU.clear();
    posV.clear();

    switch(state){
        case 0:
            cout << "Detected nothing " << endl;
            trackerimage = image;
            circle(trackerimage, center, radius, CV_RGB(255,0,0));
            drawMarker(trackerimage, center,  Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);
            IBVS.Null_Control();
        break;
        case 1:
            cout << "No matches " << endl;
            //Draw keypoints tracker image: trackerimage
            drawKeypoints(image, keypoints, trackerimage, Scalar(255,255,255) ,cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            circle(trackerimage, center, radius, CV_RGB(255,0,0));
            drawMarker(trackerimage, center,  Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);
            IBVS.Null_Control();
        break;
        case 2:
            //Draw matched keypoints
            drawKeypoints(image, keypoints, trackerimage, Scalar(255,255,255) ,cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            circle(trackerimage, center, radius, CV_RGB(255,0,0));
            drawMarker(trackerimage, center,  Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);

            //Go through the matches and get the error:
            for (int i = 0; i < inlier_matches.size(); ++i)
            {
                //Get match keypoint index:
                id = inlier_matches[i].trainIdx;

                //
                if (use_vpc==1)
                {
                    //VPC acts preemptively by prediction so it needs u,v points regardless of in or out circle
                    posU.push_back(keypoints[id].pt.x);
                    posV.push_back(keypoints[id].pt.y);
                    //Calculate Radius 
                    R = sqrt((pow(keypoints[id].pt.y - center.y,2.0) + pow(keypoints[id].pt.x - center.x,2.0)));
                    //x^2 + y^2 > r^2 outside circle?
                    if (R > radius)
                    {
                        //Draw line from point drawing it to center
                        drawline(trackerimage, center, keypoints[id].pt);

                        //Determine the feature Error from circle:
                        magnitude = R - radius;
                        //deltas and position in x and y direction appended
                        deltaU.push_back(-magnitude*(keypoints[id].pt.x - center.x)/R);
                        deltaV.push_back(-magnitude*(keypoints[id].pt.y - center.y)/R);
                    }
                    else{
                        //Its in the circle no error
                        //deltas and position in x and y direction appended
                        deltaU.push_back(0);
                        deltaV.push_back(0);
                    }


                }
                else{
                    //Original classical, only accumulate U and V if there is an error
                    //Calculate Radius 
                    R = sqrt((pow(keypoints[id].pt.y - center.y,2.0) + pow(keypoints[id].pt.x - center.x,2.0)));
                    //x^2 + y^2 > r^2 outside circle?
                    if (R > radius)
                    {
                        //Draw line from point drawing it to center
                        drawline(trackerimage, center, keypoints[id].pt);

                        //Determine the feature Error from circle:
                        magnitude = R - radius;
                        //deltas and position in x and y direction appended
                        deltaU.push_back(-magnitude*(keypoints[id].pt.x - center.x)/R);
                        deltaV.push_back(-magnitude*(keypoints[id].pt.y - center.y)/R);
                        posU.push_back(keypoints[id].pt.x);
                        posV.push_back(keypoints[id].pt.y);
                    }
                    
                }

            }
            //COMPUTE IBVS CONTROL LAW
            IBVS.Compute_Control_law(posU,posV,deltaU,deltaV,Vteleop,center.x,center.y,radius);

            //cout << "Wc = " << IBVS.Wc.transpose() << endl;

        break;
    }
    outputImage = trackerimage;
}


//Draw a line
void Feature_tracker::drawline(const Mat img, Point start, Point end )
{
  int thickness = 2;
  int lineType = LINE_8;
  line( img,
    start,
    end,
    Scalar( 0, 0, 255 ),
    thickness,
    lineType );
}

/*
Detect keypoints and try to match then to target_keypoints
*/
void Feature_tracker::track_target_keypoints(const Mat image, std::vector<double> V_tele){

	//Convert to Grayscale before feature detection
    cvtColor(image, greyimage, CV_BGR2GRAY);

    //Clear data and update Keypoints:
    keypoints.clear(); matched1.clear(); matched2.clear(); nn_matches.clear();
    inliers1.clear(); inliers2.clear(); inlier_matches.clear(); 
    detector->detectAndCompute(greyimage, noArray(), keypoints, decKey);	

    if (keypoints.size() > 0){
        //Match keypoints with knn brute force
        matcher->knnMatch(descTarget, decKey, nn_matches, 2);

        //From those matches sort them based on distance...
        for(int i = 0; i < nn_matches.size(); i++) {
            //cout << i << endl; //SEGMENTATION FAULT after this line i = 0
            if(nn_matches[i][0].distance < nn_match_ratio * nn_matches[i][1].distance) {
                matched1.push_back(target_keypoints[nn_matches[i][0].queryIdx]);
                matched2.push_back(		  keypoints[nn_matches[i][0].trainIdx]);
            }
        }
        //If there are enough matches
        if (matched1.size() > MIN_KEYPOINTS) 
        {
            //Filter matches:
            for(unsigned i = 0; i < matched1.size(); i++) {
                new_i = static_cast<int>(inliers1.size());
                inliers1.push_back(matched1[i]);
                inliers2.push_back(matched2[i]);
                inlier_matches.push_back(DMatch(new_i, new_i, 0));
            }

            //Draw matched keypoints target image
            //draw_matches_cases(image,2);
            draw_feature_tracking(image,2,V_tele);
        }
        else{
            //Draw no matches keypoints target image
            //draw_matches_cases(image,1);
            draw_feature_tracking(image,1,V_tele);
        } 
    }
    else{
            //Draw scenario of no matches
            //draw_matches_cases(image,0);
            draw_feature_tracking(image,0,V_tele);
    } 
}


/*
Establishes the initial keypoints seen at the start of visual servoing given an image
Outputs true if there are enough keypoints detected.
*/
void Feature_tracker::establish_target_keypoints(const Mat image){

	//Convert to Grayscale before feature detection
    cvtColor(image, greyimage, CV_BGR2GRAY);

    //Get Keypoints:
    keypoints.clear(); 
    target_keypoints.clear(); 
    Mat desc_1;
    detector->detectAndCompute(greyimage, noArray(), keypoints, decKey);

    //Define circle parameters:
    height = image.cols; 
    width = image.rows;
    radius = height/4;
    center = Point(height/2, width/2); 
    
    //Filter out points that are not inside the circle
    for (int i = 0; i < keypoints.size(); i++)
    {
      //x^2 + y^2 <= r^2 inside circle?
      if ((pow(keypoints[i].pt.y - center.y,2.0) + pow(keypoints[i].pt.x - center.x,2.0)) <= pow(radius,2.0))
      {
        //Append keypoint to Target keypoints 
        target_keypoints.push_back(keypoints[i]); 
        //Also append corresponding descriptor
        if (desc_1.empty())
        {
            //First Target descriptor
            desc_1 = decKey.row(i).clone();
        }
        else{
            //Concatenate descriptors cloumn wise after first
            vconcat(desc_1,decKey.row(i).clone(),desc_1);
        }
      }
    } 
    //Replace Target descriptor
    descTarget = desc_1.clone();
    
    //Draw Keypoints, center and circle
    drawKeypoints(image, target_keypoints, outputImage, Scalar(255,255,255) ,cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    circle(outputImage, center, radius, CV_RGB(255,0,0));
    drawMarker(outputImage, center,  Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);

    if (target_keypoints.size() > MIN_KEYPOINTS)
    {
    	established = true;
    	cout << "\nTarget Keypoints established!" << endl;
    	initial_image = image;
        cout << "keypoints=\n" << keypoints.size() << endl;
        //cout << "Descriptor=\n" << decKey.size() << endl;
        cout << "Target keypoints=\n" << target_keypoints.size() << endl;
        //cout << "Target Descriptor=\n" << descTarget.size() << endl;
    }
    else
    {
    	established = false;
    }
}

/*
Draws the matches and concatenates the images if needed
*/
void Feature_tracker::draw_matches_cases(const Mat image, int state){
    switch(state){
        case 0:
            cout << "Detected nothing " << endl;
            drawKeypoints(initial_image, target_keypoints, initial_imageLabelled, Scalar(255,255,255) ,cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            circle(initial_imageLabelled, center, radius, CV_RGB(255,0,0));
            drawMarker(initial_imageLabelled, center,  Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);
            circle(image, center, radius, CV_RGB(255,0,0));
            //drawMarker(image, center,  Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);
            hconcat(initial_imageLabelled, image, outputImage);
        break;
        case 1:
            cout << "No matches " << endl;
            //Draw keypoints target image
            drawKeypoints(initial_image, target_keypoints, initial_imageLabelled, Scalar(255,255,255) ,cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            circle(initial_imageLabelled, center, radius, CV_RGB(255,0,0));
            drawMarker(initial_imageLabelled, center,  Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);
            //Draw keypoints tracker image: trackerimage
            drawKeypoints(image, keypoints, trackerimage, Scalar(255,255,255) ,cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            circle(trackerimage, center, radius, CV_RGB(255,0,0));
            drawMarker(trackerimage, center,  Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);
            //Combine
            hconcat(initial_imageLabelled, trackerimage, outputImage);
        break;
        case 2:
            drawKeypoints(initial_image, target_keypoints, initial_imageLabelled, Scalar(255,255,255) ,cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            circle(initial_imageLabelled, center, radius, CV_RGB(255,0,0));
            drawMarker(initial_imageLabelled, center,  Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);
            //Draw keypoints tracker image: trackerimage
            drawKeypoints(image, keypoints, trackerimage, Scalar(255,255,255) ,cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            circle(trackerimage, center, radius, CV_RGB(255,0,0));
            drawMarker(trackerimage, center,  Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);
            //EXCEMPTION IN DRAW MATCHES
            try{
                drawMatches(initial_imageLabelled, target_keypoints, trackerimage, keypoints, inlier_matches, outputImage); 
            }
            catch(...){
                cout << "EXCEMPTION in drawmatches Ignored! " << endl;
            }
        break;
    }
}
