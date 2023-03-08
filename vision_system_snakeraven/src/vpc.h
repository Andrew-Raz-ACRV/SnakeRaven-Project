#ifndef VPC_H_
#define VPC_H_
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

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/AutoDiff>

#include <iostream>

//Limit to the number of features for VPC (twice Max_features) i.e 10 points a 20 by 1 s vector
#define Max_s 20 
//Prediction Horizon
#define Np 5

using namespace Eigen;

//Visual Predictive control Class object
class vpc
{
public:
	//Initialise VPC parameters
	Matrix<double, Max_s, Max_s> Q, Q2;
	Matrix<double, 3, 3> R;
	double dt, Vmax; 

	//Region reaching criteria
	Matrix<double, 2, 1> center, Rad;
	double Radius, radius;

	//Feature error
	Matrix<double, Max_s, 1> s, s_update, e, error;
	int s_size;

	//Image Jacobians
	Matrix<double, Max_s, 6> Ls;
	double x, y, z;

	//Solution vector
	Matrix<double, 3, 1> v, dw0, dw;
	Matrix<double, 1, 1> cost;
	VectorXd U;

	//Construct class with given default properties:
	vpc();
	void get_s_v(const VectorXd&, const VectorXd&);
	void get_circle_data(double center_x,double center_y, double in_radius);
};

template <typename Derived>
void Ls_matrix(DenseBase<Derived>& Ls, const VectorXd& s, const int s_size)
{
	//Output a image jacobian matrix
	double x, y, z;
	z = 30; //mm
	//
	for (size_t i = 0; i < (0.5*s_size); i++)
	{
		//Extract Pixel coordinates note:
		x = s(2 * i);
		y = s(2 * i + 1);

		//Compute Image Jacobian Translation
		Ls(2 * i, 0) = 1.0 / z;		    Ls(2 * i, 1) = 0;	           Ls(2 * i, 2) = x / z;
		Ls(2 * i + 1, 0) = 0;		Ls(2 * i + 1, 1) = 1.0 / z;      Ls(2 * i + 1, 2) = y / z;

		//Compute Image Jacobian Orientation
		Ls(2 * i, 3) = x * y;					Ls(2 * i, 4) = -(1 + x * x);      Ls(2 * i, 5) = y;
		Ls(2 * i + 1, 3) = 1 + y * y;			Ls(2 * i + 1, 4) = -x * y;	 Ls(2 * i + 1, 5) = -x;

	}
}

template <typename Derived>
void circle_region_error(DenseBase<Derived>& error, const VectorXd& s, const VectorXd& center, const double radius, const int s_size)
{
	Vector2d Rad; double Radius;
	for (size_t i = 0; i < (0.5*s_size); i++)
	{
		Rad(0) = s(2 * i) - center(0);
		Rad(1) = s(2 * i + 1) - center(1);
		Radius = Rad.norm();
		if (Radius > radius) {
			error(2 * i) = (radius - Radius)*Rad(0) / Radius;
			error(2 * i + 1) = (radius - Radius)*Rad(1) / Radius;
		}
	}
}

// Generic functor - Think of it as a definition for a generic function handler
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{
	typedef _Scalar Scalar;
	enum {
		InputsAtCompileTime = NX,
		ValuesAtCompileTime = NY
	};
	typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

	int m_inputs, m_values;

	Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
	Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

	int inputs() const { return m_inputs; }
	int values() const { return m_values; }

};

//VPC cost function structure
struct cost_function : Functor<double>
{
	int operator()(const Eigen::VectorXd &U, Eigen::VectorXd &fvec) const
	{
		//Note outputs >= inputs
		//object vpc a has to be constant! i.e. values in it cannot change here!
		
		//Initialise feature update s_k
		Matrix<double, Max_s, 1> s_k, e;
		s_k = a.s;
		e = MatrixXd::Zero(Max_s, 1);
		Matrix<double, Max_s, 6> Ls = MatrixXd::Zero(Max_s, 6);

		//Go through prediction horizon
		for (size_t i = 0; i < Np; i++)
		{
			//Image Jacobian
			Ls_matrix(Ls, s_k, a.s_size);
			//std::cout << "Ls = " << std::endl << Ls << std::endl;

			//Prediction step
			s_k.block(0, 0, a.s_size, 1) = s_k.block(0, 0, a.s_size, 1) + Ls.block(0, 0, a.s_size, 3)*U.block(3 * i, 0, 3, 1)*a.dt + Ls.block(0, 3, a.s_size, 3)*a.v*a.dt;
			//std::cout << "s_k = " << std::endl << s_k << std::endl;

			//Compute expected error
			circle_region_error(e, s_k, a.center, a.radius, a.s_size);
			//std::cout << "e(k) = " << std::endl << e << std::endl;

			//Compute terms to minimise
			fvec(3 * i) = (U.block(3 * i, 0, 3, 1).transpose() * a.R * U.block(3 * i, 0, 3, 1)).value();
			fvec(3 * i + 1) = (e.block(0, 0, a.s_size, 1).transpose()*a.Q.block(0, 0, a.s_size, a.s_size)*e.block(0, 0, a.s_size, 1)).value();
			fvec(3 * i + 2) = 0;
		}
		return 0;
	}
	vpc a;

	int inputs() const { return 3 * Np; } // There are two parameters of the model
	int values() const { return 3 * Np; } // The number of outputs
};

//VPC cost function structure faster
struct cost_function_v2 : Functor<double>
{

	int operator()(const Eigen::VectorXd &U, Eigen::VectorXd &fvec) const
	{
		//Note outputs >= inputs
		//object vpc a has to be constant! i.e. values in it cannot change here!

		//Initialise feature update s_k
		Matrix<double, Max_s, 1> s_k, e;
		s_k = a.s;
		e = MatrixXd::Zero(Max_s, 1);
		Matrix<double, Max_s, 6> Ls = MatrixXd::Zero(Max_s, 6);

		//Go through prediction horizon
		for (size_t i = 0; i < Np; i++)
		{
			//Image Jacobian
			Ls_matrix(Ls, s_k, a.s_size);
			//std::cout << "Ls = " << std::endl << Ls << std::endl;

			//Prediction step
			s_k.block(0, 0, a.s_size, 1) = s_k.block(0, 0, a.s_size, 1) + Ls.block(0, 0, a.s_size, 3)*U.block(3 * i, 0, 3, 1)*a.dt + Ls.block(0, 3, a.s_size, 3)*a.v*a.dt;
			//std::cout << "s_k = " << std::endl << s_k << std::endl;

			//Compute expected error
			circle_region_error(e, s_k, a.center, a.radius, a.s_size);
			//std::cout << "e(k) = " << std::endl << e << std::endl;

			//Compute terms to minimise note each element sum(fvec^2)
			fvec.block((Max_s + 3) * i, 0, Max_s, 1) = a.Q2*e;
			fvec.block((Max_s + 3) * i + Max_s, 0, 3, 1) = a.R*U.block(3 * i, 0, 3, 1);
		}
		return 0;
	}
	vpc a;
	int inputs() const { return 3 * Np; } // There are two parameters of the model
	int values() const { return ((3 + Max_s) * Np); } // The number of outputs
};

#endif