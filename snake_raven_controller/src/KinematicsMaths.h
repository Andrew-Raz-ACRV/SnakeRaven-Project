#ifndef KINEMATICSMATHS_H_
#define KINEMATICSMATHS_H_
/*
kinematics maths functions declarations

author: Andrew Razjigaev 2021
*/

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

using namespace Eigen;
using namespace std;

#define PI           3.14159265358979323846  /* pi */

/*
Kinematics Math Functions for the Snake Raven Manipulator

It defines constants PI 

It includes these functions with support from Eigen:
- dh 					- DH parameter matrix function: dh
- isodd 				- is odd logic function: isodd
- deg2rad 				- degrees to radians function: deg2rad
- skew 					- Skew symmetric matrix function: skew
- vex 					- Inverse skew (vex) function: vex
- trans2dx 				- transformation to delta vector function: trans2dx
- dx2trans 				- delta vector to transform function: dx2trans
- Tnorm 				- Normalise a transformation matrix to ensure the rotation matrix is true: Tnorm
- cap_mag 				- cap the magnitude of a vector function: cap_mag
- dampedLeastSquares 	- Compute the damped least squares pseudo inverse of a Jacobian: dampedLeastSquares
- applyJointLimits 		- Restrict a joint vector to be within its limits: applyJointLimits
- TransformPoints 		- Transform Points to a coordinate frame with: TransformPoints

author: Andrew Razjigaev 2021
*/

//Kinematics functions:
MatrixXd dh(const double alpha, const double a, const double d, const double theta);

bool isodd(double v);

double deg2rad(double degrees);

double rad2deg(double radians);

MatrixXd skew(const VectorXd& v);

VectorXd vex(const MatrixXd& S);

MatrixXd trans2dx(const MatrixXd& T0, const MatrixXd& T1);

VectorXd cap_mag(const VectorXd& V, const double magnitude);

MatrixXd applyJointLimits(const MatrixXd& q, const MatrixXd& ql, const MatrixXd& qu);

bool JointLimitcheck(const MatrixXd& q, const MatrixXd& ql, const MatrixXd& qu);

MatrixXd dampedLeastSquares(const MatrixXd& J, const MatrixXd& q, const MatrixXd& ql, const MatrixXd& qu);

MatrixXd TransformPoints(const MatrixXd& T, const MatrixXd& p);

MatrixXd dx2trans(const MatrixXd& dx);

MatrixXd Tnorm(const MatrixXd& T);

MatrixXd logMatrix(const MatrixXd& M); //Hand Eye math function


#endif