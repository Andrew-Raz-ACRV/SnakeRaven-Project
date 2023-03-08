#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//ROS publish data with std msg:
#include "std_msgs/Float32MultiArray.h"

//feature tracking class for IBVS
#include "Feature_tracker.h"
//Aruco marker tracking class for IBVS
#include "aruco_tracker.h"
//Recording data:
#include <iostream>
#include <string>
#include <stdio.h>
#include <inttypes.h>


using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

//State machine modes
enum modes {
  idle,
  ArUco_tracking,
  IBVS_inactive,
  IBVS_active
};

class ImageConverter
{
public:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber camera_data_sub;
  ros::Subscriber snake_mode_sub;
  ros::Publisher aruco_transform_pub;
  ros::Publisher camera_velocity_pub;

  //Camera Calibration data and properties:
  Mat cameraMatrix, distCoeffs, OUTPUT_IMAGE;
  float scale; //OUTPUT_IMAGE scale
  float height, width; //Image Dimensions in pixels
  float image_sensor[2]; //Dimensions of pixel array in mm
  int frame; //Frame counter
  int frame_count;

  //State modes:
  modes state; modes new_state; bool status_report;

  //Teleop command from subscriber
  vector<double> V_teleop;
  int teleop_hold;
  int teleop_period;

  //Feature Trackers
  Feature_tracker tracker;
  aruco_tracker aruco;

  //Record data into file:
  FILE * pFile;
  //stringstream ss;
  //string str;
  bool RECORDING_CAMERA;

  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/cv_camera/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    camera_data_sub = nh_.subscribe("/cv_camera/camera_info",1,
      &ImageConverter::getCamerainfo,this);

    snake_mode_sub = nh_.subscribe("/snakeraven_mode",1,
      &ImageConverter::getsnake_mode,this);
    
    //Publish aruco pose and IBVS camera velocity
    aruco_transform_pub = nh_.advertise<std_msgs::Float32MultiArray>("/image_converter/aruco_pose",1);
    camera_velocity_pub = nh_.advertise<std_msgs::Float32MultiArray>("/image_converter/camera_velocity",1);

    //Initalise variables for the image process:
    frame = 0;
    state = idle;
    new_state = idle;
    status_report = true;
    scale = 2.5; //scale output image

    //initialise Vteleop and teleop hold limit
    for (int i = 0; i < 3; ++i)
    {
    	V_teleop.push_back(0);
    }
    teleop_hold = 0;
    teleop_period = 20;

    //Dimensions of pixel array in mm
    image_sensor[0] = 0.9; image_sensor[1] = 0.9; 

    cv::namedWindow(OPENCV_WINDOW);


    //Set up file to record feature error over time:
    RECORDING_CAMERA = true; frame_count = 0;
    if (RECORDING_CAMERA)
	{
		this->pFile = fopen("FeatureTrackerIBVSData.csv","w");
		//Odd is u and even is v value
		fprintf(pFile, "sec, nsec, Error1, Error2, Error3, Error4, Error5, Error6, Error7, Error8, Error9, Error10, Error11, Error12, Error13, Error14, Error15, Error16, Error17, Error18, Error19, Error20, \n");
	}
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void record_data_csv()
  {
  	//We want to know the feature error vector over time 
	//Time
	fprintf(pFile, "%"PRIu32", %"PRIu32",", ros::Time::now().sec, ros::Time::now().nsec);

	//Convert Eigen matrices to 12 values:
	for (int i = 0; i < (2 * Max_features); i++)
	{
		fprintf(pFile, "%.4f,", tracker.IBVS.Error(i));
	}

	//End line
	fprintf(pFile, "\n");
  }

  void getCamerainfo(const sensor_msgs::CameraInfo msg)
  {
    //Get Camera Intrinsics Matrix
    cameraMatrix = (Mat_<double>(3,3) << 
      msg.K[0], msg.K[1], msg.K[2], 
      msg.K[3], msg.K[4], msg.K[5], 
      msg.K[6], msg.K[7], msg.K[8]);
    //cout << "K = " << cameraMatrix << endl;

    //Get Distortion Coefficients
    distCoeffs = (Mat_<double>(1,5) <<
      msg.D[0], msg.D[1], msg.D[2], msg.D[3], msg.D[4]);
    //cout << "D = " << distCoeffs << endl;
    height = msg.height; width = msg.width;
  }

  void getsnake_mode(std_msgs::Float32MultiArray msg)
  {
  	//Get mode from snakeraven controller to know how to process the image
  	if (msg.data[0]==5)
  	{
  		new_state = IBVS_inactive;
  	}
  	else if (msg.data[0]==5.5)
  	{
  		new_state = IBVS_active;
  	}
  	else if (msg.data[0]==4)
  	{
  		new_state = ArUco_tracking;
  	}
  	else{
  		new_state = idle;
  	}

  	//Teleop data processing:
  	//Update if non zero input or if its been 'teleop_period' values;
  	if ((msg.data[1]==0)&&(msg.data[2]==0)&&(msg.data[3]==0))
  	{
  		//Don't update for a period for VPC to make better predictions
  		teleop_hold++;
  		if (teleop_hold > teleop_period)
  		{
	  		//Get teleop command Vc from the next array values 1,2,3
	  		V_teleop.clear();
		  	for (int i = 1; i < 4; ++i)
		  	{
		  		V_teleop.push_back(msg.data[i]);
		  	}
		  	teleop_hold = 0;
  		}
  	}
  	else{
  		//Get teleop command Vc from the next array values 1,2,3
  		V_teleop.clear();
	  	for (int i = 1; i < 4; ++i)
	  	{
	  		V_teleop.push_back(msg.data[i]);
	  	}
	  	teleop_hold = 0;
  	}


  	if (new_state!=state)
  	{
  		status_report = true;
  		state = new_state;
  	}
  }

  void mode_status()
  {
    if (status_report){
      switch(state)
      {
        case idle:
        {
          cout << "Mode status: idle" << endl; break;
        }
        case ArUco_tracking:
        {
          cout << "Mode status: ArUco_tracking" << endl; break;
        }
        case IBVS_inactive:
        {
          cout << "Mode status: IBVS_inactive" << endl; break;
        }
        case IBVS_active:
        {
          cout << "Mode status: IBVS_active" << endl; break;
        }
      }
    }
    status_report = false;
  }


  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //Image Processing here referring to image: cv_ptr->image

    //Initalise Tracker messages:
    std_msgs::Float32MultiArray msg_camera_velocity;
    std_msgs::Float32MultiArray msg_aruco_pose;
    msg_camera_velocity.data.clear();
    msg_aruco_pose.data.clear();

    //Get the current snake_raven_state to decide what vision system to do?
    mode_status();
    switch(state)
    {
      case idle:
      {
        //Neglect all detections and just show plain image
        aruco.null_detection();
        tracker.IBVS.Null_Control(); tracker.established = false;
        //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::resize(cv_ptr->image, OUTPUT_IMAGE, cv::Size(), scale, scale);
        cv::imshow(OPENCV_WINDOW, OUTPUT_IMAGE);
        break;
      }
      case IBVS_active:
      {
        //START FEATURE Tracking
        if (tracker.established)
        {
          //Track the features moving from the circle
          tracker.track_target_keypoints(cv_ptr->image, V_teleop);
          //Show camera velocity:
          //cout << "Wc = " << endl << tracker.IBVS.Wc.transpose() << endl;
        }
        else
        {
          //Determine desired features to keep inside the circle
          tracker.establish_target_keypoints(cv_ptr->image);
          if (tracker.established)
          {
            //Get camera constants for IBVS:
            tracker.IBVS.Get_Constants(cameraMatrix, height, width, image_sensor[0], image_sensor[1]);
            //Begin Tracking!
            cout << "FEATURES Tracking begins!" << endl;
          }
        }

        //Neglect other detections and show feature tracker image:
        aruco.null_detection();
        //cv::imshow(OPENCV_WINDOW, tracker.outputImage);
        cv::resize(tracker.outputImage, OUTPUT_IMAGE, cv::Size(), scale, scale);
        cv::imshow(OPENCV_WINDOW, OUTPUT_IMAGE);

        if (RECORDING_CAMERA)
        {
          record_data_csv();
          //record frames:
          stringstream ss;
          ss << frame_count;
          string str = ss.str();
          cv::imwrite("Frame_"+str+".jpg", tracker.outputImage);
          frame_count++;
        }

        break;
      }
      case IBVS_inactive:
      {
        //Just display the IBVS view screen 
        tracker.passive_mode(cv_ptr->image);

        //Neglect other detections and show feature tracker image:
        aruco.null_detection();
        //cv::imshow(OPENCV_WINDOW, tracker.outputImage);
        //cv::resize(tracker.outputImage, OUTPUT_IMAGE, cv::Size(), scale, scale); //SHOW CIRCLE
        cv::resize(cv_ptr->image, OUTPUT_IMAGE, cv::Size(), scale, scale); //NOT SHOW CIRCLE
        cv::imshow(OPENCV_WINDOW, OUTPUT_IMAGE);

        break;
      }
      case ArUco_tracking:
      {
        //Get aruco pose
        aruco.Get_pose(cv_ptr->image,cameraMatrix, distCoeffs,frame);
        //Neglect other tracker
        tracker.IBVS.Null_Control(); tracker.established = false;
        //show image:
        //cv::imshow(OPENCV_WINDOW, aruco.output_image);
        cv::resize(aruco.output_image, OUTPUT_IMAGE, cv::Size(), scale, scale);
        cv::imshow(OPENCV_WINDOW, OUTPUT_IMAGE);
        break;
      }
    }

    cv::waitKey(3);

    //CREATE AND PUBLISH MESSAGES:
    for (int i=0; i<3;i++){
      msg_camera_velocity.data.push_back(tracker.IBVS.Wc(i));
      tracker.IBVS.Wc(i) = 0; //Reset for next loop
    }

    for (int i=0; i<12;i++){
      msg_aruco_pose.data.push_back(aruco.transform[i]);
    }

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
    //Publish Aruco Pose
    aruco_transform_pub.publish(msg_aruco_pose);
    //Publish Camera Velocity
    camera_velocity_pub.publish(msg_camera_velocity);

    //Update Frame count:
    frame++;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
