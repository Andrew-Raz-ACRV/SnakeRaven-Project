#ifndef HANDEYE_CALIBRATOR_H_
#define HANDEYE_CALIBRATOR_H_
/*
Hand Eye Calibration Class and math functions declarations

author: Andrew Razjigaev 2021
*/

#include "KinematicsMaths.h"

//#include <unsupported/Eigen/MatrixFunctions> //Just to use .pow()
//#include <sstream>
//#include <fstream>
//#include <string>
//#include <vector>

using namespace Eigen;
using namespace std;

/*
Hand Eye Class

This file defined the class Hand Eye Calibrator and its functions:
- navy_calibration

To append data do:
HandEye_calibrator H;
H.Hmarker2world.block(4 * i, 0, 4, 4) = Twg;
H.Hgrid2cam.block(4 * i, 0, 4, 4) = Tct;

author: Andrew Razjigaev 2021
*/

//The number of views set for calibration procedure:
#define Number_of_Views 100

//State machine modes
enum hand_eye_process {
	reset_arm,
	collect,
	compute
};

//Save to file function
//void vectorToFile(char *name, vector<double> *a);
//Get vector from filefunction
//vector<double> fileToVector(const char *name);


//Hand Eye Class
class HandEye_calibrator
{
public:
	int N; //Number of Views/data
	int ii; //Current frame iterator

	//Hand Eye calibration Data Matrices AX=XB
	Matrix<double, 4 * Number_of_Views, 4> Hmarker2world, Hgrid2cam;
	Matrix<double, 4 * (Number_of_Views - 1), 4> A, B, Hgij, Hcij;
	Matrix<double, 3 * (Number_of_Views - 1), 3> Ra, C, Atsai;
	Matrix<double, 3 * (Number_of_Views - 1), 1> Ta, Tb, d, b, err;

	//Other variables
	Matrix3d M, Rx, Rcg, R1, R2;
	Matrix<double, 3, 1> alpha, beta, Tx, Pgij, Pcij, Pcg_prime, Pcg, Tcg, rncij, rngij;
	Matrix4d Hcam2marker; //solution
	double residus_TSAI_rotation, residus_TSAI_translation;

	//constructor
	HandEye_calibrator();

	//Save to file and retrieve
	//void Save_HandEye();
	//void Retrieve_HandEye();

	//Hand Eye calibration Function:
	//MatrixXd navy_calibration(); //Euclidean Method - DO NOT USE BUG
	MatrixXd TSAIleastSquareCalibration(); //TSAI method
};


#endif