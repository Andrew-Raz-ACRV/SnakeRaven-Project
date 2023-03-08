#include "vpc.h"

/*
Visual Predictive Control visual servo class for SnakeRaven Orientation control
author: Andrew Razjigaev 2022
- get_s_v 				- clearly gets feature vector s = [x y ... ]' and v = [ 0 0 0] teleop command.
- Ls_matrix				- computes full image jacobian given s
- circle_region_error   - computes the error vector for s only during predictions
- Functor 				- generic function handler for optimising the cost function
- cost_function 		- vpc cost function original version
- cost_function_v2 		- vpc cost function a faster implementation for Eign's NL optimiser Levenburg Marquardt
*/

vpc::vpc() {

	//MPC parameters
	Q = 10 * MatrixXd::Identity(Max_s, Max_s);
	Q2 = pow(10, 0.5) * MatrixXd::Identity(Max_s, Max_s);
	R = MatrixXd::Identity(3, 3);
	dt = 1.0 / 30;
	Vmax = 0.05;
	z = 1;

	//Region reaching
	center << 1, 1;
	radius = 1;
	error = MatrixXd::Zero(Max_s, 1);
	s = MatrixXd::Zero(Max_s, 1);
	e = MatrixXd::Zero(Max_s, 1);
	s_update = MatrixXd::Zero(Max_s, 1);
	cost << 0;

	//Initialise Jacobians
	Ls = MatrixXd::Zero(Max_s, 6);

	//Initial solution 
	dw0 = MatrixXd::Zero(3, 1);
	dw = MatrixXd::Zero(3, 1);
	v = MatrixXd::Zero(3, 1);
	U = VectorXd::Zero(3 * Np);
}

void vpc::get_s_v(const VectorXd& p, const VectorXd& input)
{
	//parse the feature data
	s_size = p.size();
	s.block(0, 0, s_size, 1) = p;
	s_update.block(0, 0, s_size, 1) = p;

	//parse the teleop translation
	v = input;

	//Debugging
	//std::cout << "s_k = " << p.transpose() << std::endl;
	//Matrix<double, Max_s, 1> error_k = MatrixXd::Zero(Max_s, 1);
	//circle_region_error(error_k, s, center, radius, s_size);
	//std::cout << "e_k = " << error_k.transpose() << std::endl;
}

void vpc::get_circle_data(double center_x,double center_y, double in_radius)
{
	//parse the circle data to predict error
	radius = in_radius;
	center << center_x, center_y;

	//debugging
	//std::cout << "r = " << radius << std::endl;
	//std::cout << "c = " << center.transpose() << std::endl;

}

