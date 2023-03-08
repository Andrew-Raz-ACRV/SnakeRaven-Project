#include "SnakeRaven.h"

/*
SnakeRaven Class

This file defined the class SnakeRaven and its functions:
- Constructor with default properties and gene paramters
- Forward Kinematics for a generic SnakeRaven
- Jacobian matrix for a generic SnakeRaven
- Get Tendon Lengths for a for a generic SnakeRaven using
 - GetTendonPoints
 - AppendTendonMeasurements
- q2Motor_angles conversion function that computes rotation of the Raven motors


author: Andrew Razjigaev 2019
*/

//Snake Raven Constructor Function:
SnakeRaven::SnakeRaven() {
	//Design Parameter Construction
	// number of modules
	m = Max_m;

	//Degrees of Freedom:
	DOF = 3 + 2 * m;//number of DOF
	da = 10; //mm for adaptor capstan diameter //10 
	ra = da / 2.0; //mm radius of capstan adaptor
	q = MatrixXd::Zero(DOF, 1); // Joint vector

	//Parameter sets:
	w = 4;//mm
	isrightarm = true;

	//Weighting matrix for Damped least squares
	W.diagonal() << 1, 1, 1, 5, 5, 5;

	//Calibration constants
	rate << -1*MatrixXd::Ones(DOF, 1); //Reverse rotation
	rate.block(0,0,3,1) = -1*rate.block(0,0,3,1); //But not reverse for first 3 DOF
	offset << MatrixXd::Zero(DOF, 1);

	if (m == 1) {
		//Single module
		alpha << 1.24; //radians
		n << 3; //integers
		d << 1.62; //mm
		//Initial Calibration configuration
		if (isrightarm){
			q << deg2rad(-39.5967), deg2rad(-77.9160), 0, 0, 0;
		}
		else{
			q << deg2rad(39.5967), deg2rad(-102.0840), 0, 0, 0;
		}
	}
	else if (m == 2) {
		//Double module
		alpha << 0.20, 0.88; //1.39, 1.18; //radians
		n << 3, 3; //1, 3; //integers
		d << 1, 1; //6, 0.41; //mm
		//Initial Calibration configuration
		if (isrightarm){
			q << deg2rad(-39.5967), deg2rad(-77.9160), 0, 0, 0, 0, 0;
		}
		else{
			q << deg2rad(39.5967), deg2rad(-102.0840), 0, 0, 0, 0, 0;
		}
	}

	//Tool Transform is a short z-axis translation
	Tool = Translation3d(0, 0, 5);

	//Get Neutral Tendon lengths
	q_temp = q;
	q = MatrixXd::Zero(DOF, 1);
	dL0 = GetTendonLengths();
	q = q_temp;
	
	//Run all functions
	Tend = FK(); //initial tool pose
	J = Jacobian(); //Initial Jacobian matrix
	mv = q2Motor_angles(); //Current Motor values
    mv_pre = mv; //previous motor values
    q_desired = MatrixXd::Zero(DOF, 1); //Record of the desired joint angle

    //motor value history initialisation
    for (int i = 0; i < history; i++)
    {
    	mv_history.block(0,i,DOF,1) = mv_pre;
    }

	//Automate Joint limit definitions:

	//Joint Limit definition
	ql(0) = q(0)-deg2rad(60); ql(1) = q(1)-deg2rad(45); ql(2) = -150;//mm
	qu(0) = q(0)+deg2rad(60); qu(1) = q(1)+deg2rad(45); qu(2) = 150;//mm

	//Variable Module Joint limits:
	for (int i = 0; i < m; i++)
	{
		double theta_max = alpha(i)*n(i) / 2; //Maximum Bending in pan/tilt for module
		//Append pan and tilt limits:
		ql(3 + 2 * i) = -theta_max;		ql(4 + 2 * i) = -theta_max;
		qu(3 + 2 * i) = theta_max;		qu(4 + 2 * i) = theta_max;
	}

}

//Use this function to PUT PARAMETERS IN HERE
void SnakeRaven::initialisation(const double a1, const int n1, const double d1, const double a2, const int n2, const double d2, const bool right)
{
		//Design Parameter Construction
	// number of modules
	m = Max_m;

	//Degrees of Freedom:
	DOF = 3 + 2 * m;//number of DOF
	da = 10; //mm for adaptor capstan diameter //10 
	ra = da / 2.0; //mm radius of capstan adaptor
	q = MatrixXd::Zero(DOF, 1); // Joint vector

	//Parameter sets:
	w = 4;//mm
	isrightarm = right;

	//Weighting matrix for Damped least squares
	W.diagonal() << 1, 1, 1, 5, 5, 5;

	//Calibration constants
	rate << -1*MatrixXd::Ones(DOF, 1); //Reverse rotation
	rate.block(0,0,3,1) = -1*rate.block(0,0,3,1); //But not reverse for first 3 DOF
	offset << MatrixXd::Zero(DOF, 1);

	if (m == 1) {
		//Single module
		alpha << a1; //radians
		n << n1; //integers
		d << d1; //mm
		//Initial Calibration configuration
		if (isrightarm){
			q << deg2rad(-39.5967), deg2rad(-77.9160), 0, 0, 0;
		}
		else{
			q << deg2rad(39.5967), deg2rad(-102.0840), 0, 0, 0;
		}
	}
	else if (m == 2) {
		//Double module
		alpha << a1, a2; //1.39, 1.18; //radians
		n << n1, n2; //1, 3; //integers
		d << d1, d2; //6, 0.41; //mm
		//Initial Calibration configuration
		if (isrightarm){
			q << deg2rad(-39.5967), deg2rad(-77.9160), 0, 0, 0, 0, 0;
		}
		else{
			q << deg2rad(39.5967), deg2rad(-102.0840), 0, 0, 0, 0, 0;
		}
	}

	//Tool Transform is a short z-axis translation
	Tool = Translation3d(0, 0, 5);

	//Get Neutral Tendon lengths
	q_temp = q;
	q = MatrixXd::Zero(DOF, 1);
	dL0 = GetTendonLengths();
	q = q_temp;
	
	//Run all functions
	Tend = FK(); //initial tool pose
	J = Jacobian(); //Initial Jacobian matrix
	mv = q2Motor_angles(); //Current Motor values
    mv_pre = mv; //previous motor values
    q_desired = MatrixXd::Zero(DOF, 1); //Record of the desired joint angle

    //motor value history initialisation
    for (int i = 0; i < history; i++)
    {
    	mv_history.block(0,i,DOF,1) = mv_pre;
    }

	//Automate Joint limit definitions:

	//Joint Limit definition
	ql(0) = q(0)-deg2rad(60); ql(1) = q(1)-deg2rad(45); ql(2) = -150;//mm
	qu(0) = q(0)+deg2rad(60); qu(1) = q(1)+deg2rad(45); qu(2) = 150;//mm

	//Variable Module Joint limits:
	for (int i = 0; i < m; i++)
	{
		double theta_max = alpha(i)*n(i) / 2; //Maximum Bending in pan/tilt for module
		//Append pan and tilt limits:
		ql(3 + 2 * i) = -theta_max;		ql(4 + 2 * i) = -theta_max;
		qu(3 + 2 * i) = theta_max;		qu(4 + 2 * i) = theta_max;
	}
}

//Reset q to initial
void SnakeRaven::reset_q()
{
	//Reset q vector
	if (m == 1) {
		//Initial Calibration configuration
		if (isrightarm){
			q << deg2rad(-39.5967), deg2rad(-77.9160), 0, 0, 0;
		}
		else{
			q << deg2rad(39.5967), deg2rad(-102.0840), 0, 0, 0;
		}
	}
	else if (m == 2) {
		//Initial Calibration configuration
		if (isrightarm){
			q << deg2rad(-39.5967), deg2rad(-77.9160), 0, 0, 0, 0, 0;
		}
		else{
			q << deg2rad(39.5967), deg2rad(-102.0840), 0, 0, 0, 0, 0;
		}
	}

	//Recompute the following:
	//Get Neutral Tendon lengths
	dL0 = GetTendonLengths();
	
	//Run all functions
	Tend = FK(); //initial tool pose
	J = Jacobian(); //Initial Jacobian matrix
	mv = q2Motor_angles(); //Current Motor values
	mv_pre = mv; // previous motor values

	//motor value history initialisation
	for (int i = 0; i < history; i++)
    {
    	mv_history.block(0,i,DOF,1) = mv_pre;
    }
}

//this is for an average filter of motor values
MatrixXd SnakeRaven::get_mv_pre_ave()
{
	//mv_pre was read by the sensor
	//shift the previous values right in the matrix columns
	mv_history.block(0,1,DOF,history-1) = mv_history.block(0,0,DOF,history-1);

	//append the current mv_pre to the start
	mv_history.block(0,0,DOF,1) = mv_pre;

	//Compute the average
	mv_pre_ave = mv_history.rowwise().mean();

	return mv_pre_ave;
}

//Forward Kinematics Function
MatrixXd SnakeRaven::FK()
{
	//Start transform from Raven Base:
	double La12 = deg2rad(75); double La23 = deg2rad(52);
	Matrix4d RCM, T1, T2, T3;

	if (isrightarm == true) {
		//Right arm
		RCM << 0, 0, -1, -300.71, 0, 1, 0, 61, 1, 0, 0, -7, 0, 0, 0, 1;
		T1 = dh(PI, 0, 0, q(0));
		T2 = dh(La12, 0, 0, q(1));
		T3 = dh(La23, 0, q(2), -PI / 2.0);
	}
	else {
		//Left arm
		RCM << 0, 0, 1, 300.71, 0, -1, 0, 61, 1, 0, 0, -7, 0, 0, 0, 1;
		T1 = dh(0, 0, 0, q(0));
		T2 = dh(La12, 0, 0, q(1));
		T3 = dh(PI - La23, 0, q(2), PI / 2.0);
	}
	//Start Kinematic Chain on Raven DOF
	Tend = RCM * T1*T2*T3;

	//Start snakebot Kinematic chain
	int first_disk = 1;

	//The pan first assumption
	bool pan_first = true;

	//Loop through each module k until m
	for (int k = 0; k < m; k++)
	{
		//Decide on number of pan and tilt joints:
		double np, nt;
		if (pan_first == true) {
			np = round(n(k) / 2.0); nt = n(k) - np;
		}
		else {
			nt = round(n(k) / 2.0); np = n(k) - nt;
		}

		//Extract joint angles for pan and tilt of module k
		double Op = q(3 + 2 * k); //pan angle
		double Ot = q(4 + 2 * k); //tilt angle

								  //Get Radius of curvature from alpha:
		double r = (w / 2.0) / sin(alpha(k));

		//Compute Pan transform:
		double r01x = 0;
		double r01y = -2 * r * (1 - cos(alpha(k))*cos(Op / (2 * np)))*sin(Op / (2 * np));
		double r01z = 2 * r * (1 - cos(alpha(k))*cos(Op / (2 * np)))*cos(Op / (2 * np));
		Affine3d T01(Translation3d(r01x, r01y, r01z) * AngleAxisd((Op / np), Vector3d::UnitX()));
		Affine3d T11(Translation3d(0, 0, d(k)));
		Affine3d Tpan = T01 * T11;

		//Compute Tilt Transform:
		double r12x = 2 * r * (1 - cos(alpha(k))*cos(Ot / (2 * nt)))*sin(Ot / (2 * nt));
		double r12y = 0;
		double r12z = 2 * r * (1 - cos(alpha(k))*cos(Ot / (2 * nt)))*cos(Ot / (2 * nt));
		Affine3d T12(Translation3d(r12x, r12y, r12z) * AngleAxisd((Ot / nt), Vector3d::UnitY()));
		Affine3d T22(Translation3d(0, 0, d(k)));
		Affine3d Ttilt = T12 * T22;

		//Kinematic Chain
		for (int i = first_disk; i <= (first_disk + n(k) - 1); i++)
		{
			if (pan_first == true) {
				if (isodd(i - (first_disk - 1))) {
					Tend = Tend * Tpan.matrix(); //pan
				}
				else {
					Tend = Tend * Ttilt.matrix(); //tilt
				}
			}
			else {
				if (isodd(i - (first_disk - 1))) {
					Tend = Tend * Ttilt.matrix(); //tilt
				}
				else {
					Tend = Tend * Tpan.matrix(); //pan
				}
			}
		}

		//Transition disk:
		if (k != (m - 1)) {
			Affine3d Tpd(AngleAxisd(deg2rad(-90 / m), Vector3d::UnitZ()));
			Tend = Tend * Tpd.matrix();
		}
		//Next section decide if it starts as a pan or tilt
		if ((pan_first == true) && (isodd(n(k)))) {
			pan_first = false;
		}
		else if ((pan_first == false) && (isodd(n(k)))) {
			pan_first = true;
		}
		else if ((pan_first == true) && (isodd(n(k)) == false)) {
			pan_first = true;
		}
		else if ((pan_first == false) && (isodd(n(k)) == false)) {
			pan_first = false;
		}
		//Update first disk for the next segment:
		first_disk = first_disk + static_cast<int>(n(k));
	}
	//Append Tool transform
	Tend = Tend * Tool.matrix();
	return Tend;
}

//Jacobian Function
MatrixXd SnakeRaven::Jacobian()
{
	//Initialise the output matrix 6 by DOF as zeros:
	MatrixXd J(6, DOF);
	J = MatrixXd::Zero(6, DOF);

	//Solve analytically by doing FK algorithm 
	//and measuring velocity of endeffector from rotation from each joint

	//Initialise measurement vectors:
	Vector3d Pg, P, wz; // endeffector Pg, point P, axis wz 

						//Endeffector Point:
	Pg = Tend.block(0, 3, 3, 1);

	//Start FK from Raven Base:
	double La12 = deg2rad(75); double La23 = deg2rad(52);
	Matrix4d T, RCM, T1, T2, T3;

	if (isrightarm == true) {
		//Right arm
		RCM << 0, 0, -1, -300.71, 0, 1, 0, 61, 1, 0, 0, -7, 0, 0, 0, 1;
		T1 = dh(PI, 0, 0, q(0));
		T2 = dh(La12, 0, 0, q(1));
		T3 = dh(La23, 0, q(2), -PI / 2.0);
	}
	else {
		//Left arm
		RCM << 0, 0, 1, 300.71, 0, -1, 0, 61, 1, 0, 0, -7, 0, 0, 0, 1;
		T1 = dh(0, 0, 0, q(0));
		T2 = dh(La12, 0, 0, q(1));
		T3 = dh(PI - La23, 0, q(2), PI / 2.0);
	}

	//Start solving Raven Jacobian columns:
	//Jq1
	T = RCM * T1;
	P = T.block(0, 3, 3, 1);
	wz = T.block(0, 0, 3, 3) * Vector3d::UnitZ();
	J.block(0, 0, 3, 1) = wz.cross(Pg - P);
	J.block(3, 0, 3, 1) = wz;
	//Jq2
	T = T * T2;
	P = T.block(0, 3, 3, 1);
	wz = T.block(0, 0, 3, 3) * Vector3d::UnitZ();
	J.block(0, 1, 3, 1) = wz.cross(Pg - P);
	J.block(3, 1, 3, 1) = wz;
	//Jq3
	T = T * T3;
	P = T.block(0, 3, 3, 1);
	wz = T.block(0, 0, 3, 3) * Vector3d::UnitZ();
	J.block(0, 2, 3, 1) = wz;
	J.block(3, 2, 3, 1) = MatrixXd::Zero(3, 1);

	//Start solving Snakebot Jacobian columns
	int first_disk = 1; int coln;

	//The pan first assumption
	bool pan_first = true;

	//Loop through each module k until m
	for (int k = 0; k < m; k++)
	{
		//Get Radius of curvature from alpha:
		double r = (w / 2.0) / sin(alpha(k));

		//Decide on number of pan and tilt joints:
		double np, nt;
		if (pan_first == true) {
			np = round(n(k) / 2.0); nt = n(k) - np;
		}
		else {
			nt = round(n(k) / 2.0); np = n(k) - nt;
		}

		//Initialise the Apparent Jacobian Matrices for Pan and tilt for this module
		MatrixXd Ja_pan = MatrixXd::Zero(6, static_cast<int>(np));
		MatrixXd Ja_tilt = MatrixXd::Zero(6, static_cast<int>(nt));

		//Extract joint angles for pan and tilt of module k
		double Op = q(3 + 2 * k); //pan angle
		double Ot = q(4 + 2 * k); //tilt angle

								  //Compute Pan transform:
		double r01x = 0;
		double r01y = -2 * r * (1 - cos(alpha(k))*cos(Op / (2 * np)))*sin(Op / (2 * np));
		double r01z = 2 * r * (1 - cos(alpha(k))*cos(Op / (2 * np)))*cos(Op / (2 * np));
		Affine3d T01(Translation3d(r01x, r01y, r01z) * AngleAxisd((Op / np), Vector3d::UnitX()));
		Affine3d T11(Translation3d(0, 0, d(k)));
		Affine3d Tpan = T01 * T11;

		//Compute Tilt Transform:
		double r12x = 2 * r * (1 - cos(alpha(k))*cos(Ot / (2 * nt)))*sin(Ot / (2 * nt));
		double r12y = 0;
		double r12z = 2 * r * (1 - cos(alpha(k))*cos(Ot / (2 * nt)))*cos(Ot / (2 * nt));
		Affine3d T12(Translation3d(r12x, r12y, r12z) * AngleAxisd((Ot / nt), Vector3d::UnitY()));
		Affine3d T22(Translation3d(0, 0, d(k)));
		Affine3d Ttilt = T12 * T22;

		//Kinematic Chain
		for (int i = first_disk; i <= (first_disk + n(k) - 1); i++)
		{
			if (pan_first == true) {
				if (isodd(i - (first_disk - 1))) {
					T = T * Tpan.matrix(); //pan
					P = T.block(0, 3, 3, 1); wz = T.block(0, 0, 3, 3) * Vector3d::UnitX();
					coln = static_cast<int>(ceil((i - (first_disk - 1)) / 2.0)) - 1;
					Ja_pan.block(0, coln, 3, 1) = wz.cross(Pg - P);
					Ja_pan.block(3, coln, 3, 1) = wz;
				}
				else {
					T = T * Ttilt.matrix(); //tilt
					P = T.block(0, 3, 3, 1); wz = T.block(0, 0, 3, 3) * Vector3d::UnitY();
					coln = static_cast<int>(((i - (first_disk - 1)) / 2.0)) - 1;
					Ja_tilt.block(0, coln, 3, 1) = wz.cross(Pg - P);
					Ja_tilt.block(3, coln, 3, 1) = wz;
				}
			}
			else {
				if (isodd(i - (first_disk - 1))) {
					T = T * Ttilt.matrix(); //tilt
					P = T.block(0, 3, 3, 1); wz = T.block(0, 0, 3, 3) * Vector3d::UnitY();
					coln = static_cast<int>(ceil((i - (first_disk - 1)) / 2.0)) - 1;
					Ja_tilt.block(0, coln, 3, 1) = wz.cross(Pg - P);
					Ja_tilt.block(3, coln, 3, 1) = wz;
				}
				else {
					T = T * Tpan.matrix(); //pan
					P = T.block(0, 3, 3, 1); wz = T.block(0, 0, 3, 3) * Vector3d::UnitX();
					coln = static_cast<int>(((i - (first_disk - 1)) / 2.0)) - 1;
					Ja_pan.block(0, coln, 3, 1) = wz.cross(Pg - P);
					Ja_pan.block(3, coln, 3, 1) = wz;
				}
			}
		}

		//Reduce Apparent Pan and tilt Jacobians to their single degree of freedom
		if (np != 0) {
			J.block(0, 3 + 2 * k, 6, 1) = Ja_pan.rowwise().sum() / np;
		}
		if (nt != 0) {
			J.block(0, 4 + 2 * k, 6, 1) = Ja_tilt.rowwise().sum() / nt;
		}

		//Transition disk:
		if (k != (m - 1)) {
			Affine3d Tpd(AngleAxisd(deg2rad(-90 / m), Vector3d::UnitZ()));
			T = T * Tpd.matrix();
		}
		//Next section decide if it starts as a pan or tilt
		if ((pan_first == true) && (isodd(n(k)))) {
			pan_first = false;
		}
		else if ((pan_first == false) && (isodd(n(k)))) {
			pan_first = true;
		}
		else if ((pan_first == true) && (isodd(n(k)) == false)) {
			pan_first = true;
		}
		else if ((pan_first == false) && (isodd(n(k)) == false)) {
			pan_first = false;
		}
		//Update first disk for the next segment:
		first_disk = first_disk + static_cast<int>(n(k));
	}
	return J;
}

// TENDON CALCULATION FUNCTIONS 

//Tendon point function
MatrixXd GetTendonPoints(const MatrixXd& T, const double w, const double alpha, const double d, bool B, bool pan, const int k, const int M) {
	// T is the transform
	// w is the width
	// alpha is half the contact angle
	// d is the distance between curved surfaces
	// B is a logic true for end or false for start
	// pan is a logic for pan or tilt
	// k is the current segment/module e.g. 0 or 1
	// M is the total number of segments in system e.g. 2

	//Initialise output: 4 tendon points (pl pr tl tr) per module, x y z columns
	MatrixXd p = MatrixXd::Zero(4 * M, 3);

	//Get Width Radius and curve radius
	double rad = w / 2.0;
	double r = rad / sin(alpha);
	double theta;

	//Define Tendon Placement angles:
	Vector4d angles; RowVector3d po;
	// Tendon order: Pan left (-90), Pan right (90), tilt left (0), tilt right (180)
	angles << -PI / 2.0, PI / 2.0, 0, PI;

	if (pan == 1) { //pan case
		if (B == 0) { //A beginning rolling joint frame
			for (int i = k+1; i <= M; i++)
			{
				//Go around the circle and get points
				for (int j = 0; j < 4; j++)
				{
					theta = angles(j) + (i - k - 1)*deg2rad(-90 / M);
					po << rad * cos(theta), rad*sin(theta), sqrt(pow(r,2.0) - pow(rad*sin(theta),2.0)) - r * cos(alpha);
					p.block((i - 1) * 4 + j, 0, 1, 3) = po;
				}
			}
		}
		else if (B == 1) {
			for (int i = k+1; i <= M; i++)
			{
				//Go around the circle and get points
				for (int j = 0; j < 4; j++)
				{
					theta = angles(j) + (i - k - 1)*deg2rad(-90 / M);
					po << rad * cos(theta), rad*sin(theta), -sqrt(pow(r, 2.0) - pow(rad*sin(theta), 2.0)) + r * cos(alpha) - d;
					p.block((i - 1) * 4 + j, 0, 1, 3) = po;
				}
			}
		}
	}
	else if (pan == 0) { //tilt case
		if (B == 0) {
			for (int i = k+1; i <= M; i++)
			{
				//Go around the circle and get points
				for (int j = 0; j < 4; j++)
				{
					theta = angles(j) + (i - k - 1)*deg2rad(-90 / M);
					po << rad * cos(theta), rad*sin(theta), sqrt(pow(r, 2.0) - pow(rad*cos(theta), 2.0)) - r * cos(alpha);
					p.block((i - 1) * 4 + j, 0, 1, 3) = po;
				}
			}
		}
		else if (B == 1) {
			for (int i = k+1; i <= M; i++)
			{
				//Go around the circle and get points
				for (int j = 0; j < 4; j++)
				{
					theta = angles(j) + (i - k - 1)*deg2rad(-90 / M);
					po << rad * cos(theta), rad*sin(theta), -sqrt(pow(r, 2.0) - pow(rad*cos(theta), 2.0)) + r * cos(alpha) - d;
					p.block((i - 1) * 4 + j, 0, 1, 3) = po;
				}
			}
		}
	}

	//Determine Null points (when segment 1 finishes its still 0 0 0)
	MatrixXd Logic(p.rows(), 3);
	Logic = (p.rowwise()).any();

	//Transform to coordinate frame T
	p = TransformPoints(T,p);

	//Mask out the null points after the transformation
	for (int i = 0; i < p.rows(); i++)
	{
		if (Logic(i) == 0) {
			p.block(i, 0, 1, 3) = MatrixXd::Zero(1, 3);
		}
	}

	return p;
}

//Tendon Length measurement
MatrixXd AppendTendonMeasurement(const MatrixXd& Pa, const MatrixXd& Pb, const MatrixXd& L) {

	//Reshape L into a vector of series pl pr tl tr... pl pr tl tr
	MatrixXd dL = L.transpose();
	dL.resize(L.size(), 1); //Lt is now 4 by 1

	//For each tendon in Pa, Pb compute magnitude
	RowVector3d diff, A, B;

	for (int i = 0; i < Pa.rows(); i++)
	{
		//Get the vectors:
		A = Pa.block(i, 0, 1, 3); B = Pb.block(i, 0, 1, 3);
		//If they are both nonzero
		if (A.any() && B.any()) {
			//Append magnitude of tendon to total
			diff = B - A;
			dL(i) = dL(i) + diff.norm();
		}
	}	

	//Reshape and transpose to return L in original form
	dL.resize(L.cols(), L.rows());
	return dL.transpose();
}

//Tendon Length Function
MatrixXd SnakeRaven::GetTendonLengths()
{
	//Initialise Tendon Length Matrix as zeros:
	MatrixXd dLi = MatrixXd::Zero(2 * m, 2);

	//Points A and B
	MatrixXd Pa(4 * m,3); MatrixXd Pb(4 * m, 3);

	//Start Snake Kinematics from arbitrary base frame
	MatrixXd Tend = MatrixXd::Identity(4,4);

	//Start snakebot Kinematic chain
	int first_disk = 1;

	//The pan first assumption
	bool pan_first = true;

	//Loop through each module k until m
	for (int k = 0; k < m; k++)
	{
		//Decide on number of pan and tilt joints:
		double np, nt;
		if (pan_first == true) {
			np = round(n(k) / 2.0); nt = n(k) - np;
		}
		else {
			nt = round(n(k) / 2.0); np = n(k) - nt;
		}

		//Extract joint angles for pan and tilt of module k
		double Op = q(3 + 2 * k); //pan angle
		double Ot = q(4 + 2 * k); //tilt angle

								  //Get Radius of curvature from alpha:
		double r = (w / 2.0) / sin(alpha(k));

		//Compute Pan transform:
		double r01x = 0;
		double r01y = -2 * r * (1 - cos(alpha(k))*cos(Op / (2 * np)))*sin(Op / (2 * np));
		double r01z = 2 * r * (1 - cos(alpha(k))*cos(Op / (2 * np)))*cos(Op / (2 * np));
		Affine3d T01(Translation3d(r01x, r01y, r01z) * AngleAxisd((Op / np), Vector3d::UnitX()));
		Affine3d T11(Translation3d(0, 0, d(k)));
		Affine3d Tpan = T01 * T11;

		//Compute Tilt Transform:
		double r12x = 2 * r * (1 - cos(alpha(k))*cos(Ot / (2 * nt)))*sin(Ot / (2 * nt));
		double r12y = 0;
		double r12z = 2 * r * (1 - cos(alpha(k))*cos(Ot / (2 * nt)))*cos(Ot / (2 * nt));
		Affine3d T12(Translation3d(r12x, r12y, r12z) * AngleAxisd((Ot / nt), Vector3d::UnitY()));
		Affine3d T22(Translation3d(0, 0, d(k)));
		Affine3d Ttilt = T12 * T22;

		//Kinematic Chain
		for (int i = first_disk; i <= (first_disk + n(k) - 1); i++)
		{
			if (pan_first == true) {
				if (isodd(i - (first_disk - 1))) {
					//Measure Start Pa
					Pa = GetTendonPoints(Tend, w, alpha(k), d(k), 0, 1, k, m);
					Tend = Tend * Tpan.matrix(); //pan
					//Measure End Pb
					Pb = GetTendonPoints(Tend, w, alpha(k), d(k), 1, 1, k, m);
					//Append Tendon Measurement
					dLi = AppendTendonMeasurement(Pa, Pb, dLi);
				}
				else {
					//Measure Start Pa
					Pa = GetTendonPoints(Tend, w, alpha(k), d(k), 0, 0, k, m);
					Tend = Tend * Ttilt.matrix(); //tilt
					//Measure End Pb
					Pb = GetTendonPoints(Tend, w, alpha(k), d(k), 1, 0, k, m);
					//Append Tendon Measurement
					dLi = AppendTendonMeasurement(Pa, Pb, dLi);
				}
			}
			else {
				if (isodd(i - (first_disk - 1))) {
					//Measure Start Pa
					Pa = GetTendonPoints(Tend, w, alpha(k), d(k), 0, 0, k, m);
					Tend = Tend * Ttilt.matrix(); //tilt
					//Measure End Pb
					Pb = GetTendonPoints(Tend, w, alpha(k), d(k), 1, 0, k, m);
					//Append Tendon Measurement
					dLi = AppendTendonMeasurement(Pa, Pb, dLi);
				}
				else {
					//Measure Start Pa
					Pa = GetTendonPoints(Tend, w, alpha(k), d(k), 0, 1, k, m);
					Tend = Tend * Tpan.matrix(); //pan
					//Measure End Pb
					Pb = GetTendonPoints(Tend, w, alpha(k), d(k), 1, 1, k, m);
					//Append Tendon Measurement
					dLi = AppendTendonMeasurement(Pa, Pb, dLi);
				}
			}
		}

		//Transition disk:
		if (k != (m - 1)) {
			Affine3d Tpd(AngleAxisd(deg2rad(-90 / m), Vector3d::UnitZ()));
			Tend = Tend * Tpd.matrix();
		}
		//Next section decide if it starts as a pan or tilt
		if ((pan_first == true) && (isodd(n(k)))) {
			pan_first = false;
		}
		else if ((pan_first == false) && (isodd(n(k)))) {
			pan_first = true;
		}
		else if ((pan_first == true) && (isodd(n(k)) == false)) {
			pan_first = true;
		}
		else if ((pan_first == false) && (isodd(n(k)) == false)) {
			pan_first = false;
		}
		//Update first disk for the next segment:
		first_disk = first_disk + static_cast<int>(n(k));
	}
	//Return total tendon length calculation
	return dLi;
}

//Motor Values Function
VectorXd SnakeRaven::q2Motor_angles()
{
	//Set the motor angles as the last q
	mv = q;

	//Find Change in Tendon Lengths
	dLi = GetTendonLengths(); //Current lengths
	ddL = dLi - dL0; //subtract from neutral

	//Extract the wrap-up rotation
	for (int i = 0; i < (2 * m); i++)
	{
		mv(3 + i) = ddL(i, 0) / (da / 2.0);
	}

	//Go through calibration values:
	for (int i = 0; i < DOF; i++)
	{
		mv(i) = rate(i)*mv(i) + offset(i);
	}

	//Return motor position
	return mv;
}


//Motor Values Function
VectorXd SnakeRaven::Motor_angles2q()
{
	//Initialise estimate of q
	q = MatrixXd::Zero(DOF, 1);
	int iter = 0;

	//Go through calibration values:
	for (int i = 0; i < DOF; i++)
	{
		mv_pre(i) = (mv_pre(i) - offset(i)) / rate(i);
	}

	//Raven Joints are direct mapping
	for (int i = 0; i < 3; i++)
	{
		q(i) = mv_pre(i);
	}

	//Snake Joints estimation:
	//The pan first assumption
	bool pan_first = true;
	
	for (int k = 0; k < m; k++)
	{
		//Decide on number of pan and tilt joints:
		double np, nt;
		if (pan_first == true) {
			np = round(n(k) / 2.0); nt = n(k) - np;
		}
		else {
			nt = round(n(k) / 2.0); np = n(k) - nt;
		}

		//Get Radius of curvature from alpha:
		double r = (w / 2.0) / sin(alpha(k));

		//Segment-wise Coupling
		if (k > 0)
		{
			//If not first module then compute coupling of tendons
			mv_coupling = MatrixXd::Zero(DOF, 1);
			q_temp = q; //preserve original data

			//Isolate the coupling by determining it relative to neutral
			q(3 + 2 * k) = 0; //pan angle
			q(4 + 2 * k) = 0; //tilt angle

			//Find Change in Tendon Lengths
			dLi = GetTendonLengths(); //Current lengths
			ddL = dLi - dL0; //subtract from neutral

			//Get motor component from coupling
			mv_coupling(3 + 2 * k) = ddL(2 * k, 0) / (da / 2.0);
			mv_coupling(4 + 2 * k) = ddL(1 + 2 * k, 0) / (da / 2.0);

			//Subtract coupling effect elementwise
			mv_pre = mv_pre - mv_coupling;

			//Return the data back to original
			q = q_temp;
		}

		//Solving the Current Module pan and tilt angles
		MatrixXd q_(2, 1);
		q_ << q(3 + 2 * k), q(4 + 2 * k); //current estimate
		MatrixXd m_actual(2, 1);
		m_actual << mv_pre(3 + 2 * k), mv_pre(4 + 2 * k); //motor values

		if (np == 0)
		{
			//No pan no coupling
			q_(0) = 0;
			q_(1) = 2 * nt*(alpha(k) - acos(cos(alpha(k)) - (m_actual(1)*ra / (nt*w))*sin(alpha(k))));
		}
		else if (nt == 0)
		{
			//No tilt no coupling
			q_(0) = 2 * np*(alpha(k) - acos(cos(alpha(k)) - (m_actual(0)*ra / (np*w))*sin(alpha(k))));
			q_(1) = 0;
		}
		else
		{
			//Pan and tilt both couple
			//estimate by minimising motor value estimate error
			MatrixXd m_estimate(2, 1);
			m_estimate(0) = (2 * r*np*(cos(alpha(k)) - cos(alpha(k) - (q_(0) / (2 * np)))) + 2 * r*nt*(1 - cos(q_(1) / (2 * nt)))) / ra;
			m_estimate(1) = (2 * r*nt*(cos(alpha(k)) - cos(alpha(k) - (q_(1) / (2 * nt)))) + 2 * r*np*(1 - cos(q_(0) / (2 * np)))) / ra;

			//Error
			MatrixXd error;
			error = m_actual - m_estimate;

			//Actuation Jacobian
			MatrixXd Ja;
			Ja = MatrixXd::Zero(2, 2);

			//Iteratively solve q
			//cout<<"stuck here"<<endl;
			iter = 0;
			while (error.norm() > 0.001)
			{
				//watch out for this never converging due to noise
				iter++;

				//Compute actuation jacobian
				Ja(0, 0) = (-r / ra)*sin(alpha(k) - q_(0) / (2 * np));  
				Ja(0, 1) = (r / ra)*sin(q_(1) / (2 * nt));
				Ja(1, 0) = (r / ra)*sin(q_(0) / (2 * np));
				Ja(1, 1) = (-r / ra)*sin(alpha(k) - q_(1) / (2 * nt));

				//Update joint Estimate
				q_ += Ja.inverse()*error;

				//Update motor estimate
				m_estimate(0) = (2 * r*np*(cos(alpha(k)) - cos(alpha(k) - (q_(0) / (2 * np)))) + 2 * r*nt*(1 - cos(q_(1) / (2 * nt)))) / ra;
				m_estimate(1) = (2 * r*nt*(cos(alpha(k)) - cos(alpha(k) - (q_(1) / (2 * nt)))) + 2 * r*np*(1 - cos(q_(0) / (2 * np)))) / ra;

				//Update error
				error = m_actual - m_estimate;

				//In case error never converges assume 100 iterations gives descent results
				if(iter>100){
					//cout<<"Motor_angles2q failed to converge estimate in 1000 iterations..."<<endl;
					break;
				}
			}
			//cout<<"not stuck here"<<endl;
		}

		//Append solution to q
		q(3 + 2 * k) = q_(0);
		q(4 + 2 * k) = q_(1);

		//Next section decide if it starts as a pan or tilt
		if ((pan_first == true) && (isodd(n(k)))) {
			pan_first = false;
		}
		else if ((pan_first == false) && (isodd(n(k)))) {
			pan_first = true;
		}
		else if ((pan_first == true) && (isodd(n(k)) == false)) {
			pan_first = true;
		}
		else if ((pan_first == false) && (isodd(n(k)) == false)) {
			pan_first = false;
		}
	}

	//Return joint state
	return q;
}

