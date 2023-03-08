#include "KinematicsMaths.h"

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
- logMatrix				- Computes log of a Matrix as seen in the Euclidian method of hand-eye navy_calibration()

author: Andrew Razjigaev 2021
*/

//*****************KINEMATICS MATHS FUNCTIONS*********************//

MatrixXd dh(const double alpha, const double a, const double d, const double theta)
{
	//DH parameter matrix construction
	const double c0 = cos(theta);
	const double s0 = sin(theta);
	const double ca = cos(alpha);
	const double sa = sin(alpha);

	Matrix4d T;
	T << c0, -s0, 0, a,
		s0*ca, c0*ca, -sa, -d*sa,
		s0*sa, c0*sa, ca, d*ca,
		0, 0, 0, 1;
	return T;
}

bool isodd(double v) {
	//if there is a remainder after integer division by 2
	if (static_cast<int>(v) % 2) 
		return true;
	else
		return false;
}

double deg2rad(double degrees) {
	//convert degrees to radians
	return degrees * PI / 180;
}

double rad2deg(double radians) {
	//convert radians to degrees
	return radians * 180 / PI;
}

MatrixXd skew(const VectorXd& v)
{
	//Turn a 3x1 vector into a skew symmetric matrix
	Matrix3d S;
	S << 0, -v(2), v(1),
		v(2), 0, -v(0),
		-v(1), v(0), 0;
	return S;
}

VectorXd vex(const MatrixXd& S)
{
	//Turn a skew symmetric matrix into a 3x1 vector
	Vector3d v;
	v << S(2,1), S(0,2), S(1,0);
	return v;
}

MatrixXd trans2dx(const MatrixXd& T0, const MatrixXd& T1)
{
	//Turn a difference in transformation matrices into a delta vector
	Vector3d t1, t0;
	Matrix3d R0, R1;
	//Extract position:
	t0 = T0.block(0, 3, 3, 1);
	t1 = T1.block(0, 3, 3, 1);
	//Extract rotation:
	R0 = T0.block(0, 0, 3, 3);
	R1 = T1.block(0, 0, 3, 3);
	//Create 6DOF delta vector
	Matrix<double, 6, 1> dx;
	dx.block(0, 0, 3, 1) = t1 - t0;
	dx.block(3, 0, 3, 1) = vex(R1 * R0.transpose() - Matrix3d::Identity(3, 3));
	return dx;
}

//dx2trans
MatrixXd dx2trans(const MatrixXd& dx)
{
	//Turn a delta vector into a difference in transformation matrices
	Matrix<double, 4, 4> dT;
	dT = MatrixXd::Identity(4, 4);
	dT.block(0, 0, 3, 3) = Matrix3d::Identity(3, 3) + skew(dx.block(3, 0, 3, 1));
	dT.block(0, 3, 3, 1) = dx.block(0, 0, 3, 1);
	
	return dT;
}

VectorXd cap_mag(const VectorXd& V, const double magnitude)
{
	//saturates the magnitude of a vector to be a certain magnitude
	return magnitude * (V / V.norm());
}

MatrixXd applyJointLimits(const MatrixXd& q, const MatrixXd& ql, const MatrixXd& qu)
{
	//Saturates joint vector q to be within ql and qu
	MatrixXd q_ = q;

	//For each degree of freedom check the joint limits ql qu
	int DOF = q.size();
	for (int i = 0; i < DOF; i++)
	{
		if (q(i) < ql(i)) {
			q_(i) = ql(i);
			//cout<<"Lower limit "<<i<<endl;
		}
		else if (q(i) > qu(i)) {
			q_(i) = qu(i);
			//cout<<"Upper limit "<<i<<endl;
		}
	}

	//Return q but with saturation
	return q_;
}

bool JointLimitcheck(const MatrixXd& q, const MatrixXd& ql, const MatrixXd& qu)
{
	//initially assume no joint limit is reached
	bool result = false;

	//For each degree of freedom check the joint limits ql qu
	int DOF = q.size();
	for (int i = 0; i < DOF; i++)
	{
		if (q(i) < ql(i)) {
			result = true;
			//cout<<"Lower limit reached"<<endl;
		}
		else if (q(i) > qu(i)) {
			result = true;
			//cout<<"Upper limit reached"<<endl;
		}
	}

	//Return logic
	return result;
}


MatrixXd dampedLeastSquares(const MatrixXd& J, const MatrixXd& q, const MatrixXd& ql, const MatrixXd& qu)
{
	//Computes a pseudo inverse of J minimising q and the distance to limits ql and qu

	//Damping Matrix:
	int DOF = q.size();
	MatrixXd D = MatrixXd::Identity(DOF, DOF);
	/*
	//Penalty gains
	double c, p, w; 
	c = 1; p = 2; w = 1;
	
	//Generate peanalty terms in damping matrix:
	double num, den;
	for (int i = 0; i < DOF; i++)
	{
		num = 2 * q(i) - qu(i) - ql(i);
		den = qu(i) - ql(i);
		D(i, i) = c*pow((num / den),p) + (1 / w);
	}
	*/
	//Now compute the Pseudo Inverse of the Jacobian
	//MatrixXd inv_J;
	//inv_J = (J.transpose() * J + D * D).inverse() * J.transpose();

	//Matrix<double, 3 + 2 * Max_m, 6> inv_J;


	//New Method for damped least squares C++ June 2020
	MatrixXd inv_J, A, b;
	A = J.transpose() * J + D * D;
	b = J.transpose();
	inv_J = A.fullPivLu().solve(b);


	return inv_J;
}


MatrixXd TransformPoints(const MatrixXd& T, const MatrixXd& p)
{
	//Compute points p in reference transform T
	// P = T*p where p is a matrix with columns [x y z]
	int n = p.rows();
	MatrixXd P = MatrixXd::Zero(n,3);
	Vector4d ph, Pv;

	for (int i = 0; i < n; i++)
	{
		ph << p(i, 0), p(i, 1), p(i, 2), 1;
		Pv = T * ph;
		P(i, 0) = Pv(0);
		P(i, 1) = Pv(1);
		P(i, 2) = Pv(2);
	}
	return P;
}

//Tnorm
MatrixXd Tnorm(const MatrixXd& T)
{
	//Turn a delta vector into a difference in transformation matrices
	Matrix<double, 4, 4> TR;
	Matrix<double, 3, 1> n, o, a;

	//Output transform with normalised columns n,o,a
	TR = MatrixXd::Identity(4, 4);
	o = T.block(0, 1, 3, 1);
	a = T.block(0, 2, 3, 1);
	n = o.cross(a);
	o = a.cross(n);
	TR.block(0, 0, 3, 1) = cap_mag(n,1); //n
	TR.block(0, 1, 3, 1) = cap_mag(o,1); //o
	TR.block(0, 2, 3, 1) = cap_mag(a,1); //t
	TR.block(0, 3, 3, 1) = T.block(0, 3, 3, 1); //t

	return TR;
}

//Log matrix math function
MatrixXd logMatrix(const MatrixXd& M) {
	MatrixXd R, w;
	double fi;
	R = M.block(0, 0, 3, 3);
	fi = acos((R.trace() - 1) / 2);
	w = fi / (2 * sin(fi))*(R - R.transpose());
	Vector3d V(w(2, 1), w(0, 2), w(1, 0));
	return V;
}