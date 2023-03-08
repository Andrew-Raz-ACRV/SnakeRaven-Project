#ifndef IBVS_ORIENTATION_CONTROL_H_
#define IBVS_ORIENTATION_CONTROL_H_
/*
Image based visual servo class for SnakeRaven Orientation control
author: Andrew Razjigaev 2021
- Get_Constants 				- clearly gets camera matrix etc for constants f, pu, pv etc.
- orientation_image_jacobian 	- Computes image jacobian orientation partition
- Compute_Control_law			- Given the pixel position and desired velocity vector, compute camera velocity
- Null_Control					- Outputs zero velocity and error when called
*/

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
//The Visual predictive controller
#include "vpc.h"

using namespace Eigen;
using namespace std;

//Max limit for the number of point features used in IBVS for size prelocation
#define Max_features 10

//Choose the solver type 0 for classical IBVS, 1 for VPC
#define use_vpc 1

class IBVS_orientation_control
{
public:
	//Feature Error Vector:
	Matrix<double, 2 * Max_features, 1> Error, s_k;
	int curr_features;
	//Image Jacobian Constants:
	double fx, fy, pu, pv, u0, v0, lambda, Vmax;
	//Image plane feature states:
	double x, y, x_dot, y_dot;
	//Image Jacobian
	Matrix<double, 2 * Max_features, 3> Jw;

	//Orientational velocity output
	Matrix<double, 3, 1> Wc, Vc;

	//VPC object
	vpc vpc_controller;
	//Cost function object
	cost_function_v2 functor;

	//Constructor
	IBVS_orientation_control();
	//From Camera info get the constants for the Image jacobian formula
	void Get_Constants(const cv::Mat K, float height, float width, float sensor_height, float sensor_width);
	//Compute Image Jacobian:
	MatrixXd orientation_image_jacobian(float x, float y);
	//Compute Control Law:
	void Compute_Control_law(std::vector<double> posU, std::vector<double> posV, std::vector<double> deltaU, std::vector<double> deltaV, std::vector<double> Vteleop, double c1, double c2, double r);
	void Null_Control(); //Case where no control requires

};
#endif