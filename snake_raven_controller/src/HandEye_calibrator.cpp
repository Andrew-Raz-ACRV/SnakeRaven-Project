#include "HandEye_calibrator.h"
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

//CONSTRUCTOR
HandEye_calibrator::HandEye_calibrator() {
	//Set N number of views defined in header
	N = Number_of_Views;
	ii = 0; //Start at frame zero
	M = MatrixXd::Zero(3, 3); //Data matrix for rotation problem
	Hcam2marker = MatrixXd::Identity(4, 4); //Default solution identity Tcamera to gripper Tcg
}

//TSAI'S LEAST SQUARES METHOD
MatrixXd HandEye_calibrator::TSAIleastSquareCalibration()
{
	//Given the Hand to world and camera to target 4*N by 4 transforms stacked row wise,
	//it solves the Camera to Hand transform assuming the target is stationary
	
	//Ax = xB
	for (int i = 0; i < N - 1; i++)
	{
		//Stacking up Hgij and Hcij matrix getting Ra and Ta
		Hgij.block(4 * i, 0, 4, 4) = Hmarker2world.block(4 * (i + 1), 0, 4, 4).inverse()*Hmarker2world.block(4 * i, 0, 4, 4);
		Hcij.block(4 * i, 0, 4, 4) = Hgrid2cam.block(4 * (i + 1), 0, 4, 4)*Hgrid2cam.block(4 * i, 0, 4, 4).inverse();

		//Extract 
		Ra.block(3 * i, 0, 3, 3) = Hgij.block(4 * i, 0, 3, 3);
		Ta.block(3 * i, 0, 3, 1) = Hgij.block(4 * i, 3, 3, 1);
		Tb.block(3 * i, 0, 3, 1) = Hcij.block(4 * i, 3, 3, 1);
		
		//turn it into angle axis representation(rodrigues formula : P is
		//the eigenvector of the rotation matrix with eigenvalue 1
		//Pgij = AngleAxisd(Hgij.block(4 * i, 0, 3, 3)).axis();
		//Pcij = AngleAxisd(Hcij.block(4 * i, 0, 3, 3)).axis();
		//Get angle
		//theta_gij = AngleAxisd(Hgij.block(4 * i, 0, 3, 3)).angle();
		//theta_cij = AngleAxisd(Hcij.block(4 * i, 0, 3, 3)).angle();
		
		//Tsai's Modified Version:
		R1 = Hgij.block(4 * i, 0, 3, 3);
		R2 = Hcij.block(4 * i, 0, 3, 3);
		Pgij = 2 * sin(AngleAxisd(R1).angle() / 2) * AngleAxisd(R1).axis();
		Pcij = 2 * sin(AngleAxisd(R2).angle() / 2) * AngleAxisd(R2).axis();
		
		//skew(Pgij+Pcij)*x = Pcij-Pgij  which is equivalent to Ax = b
		//So we need to construct vector b and matrix A to solve this
		//overdetermined system. (Note we need >=3 Views to have at least 2
		//linearly independent inter-view relations.

		//Stacking up A and b matrix notice its 3x3 and 3x1 stacked row wise
		Atsai.block(3 * i, 0, 3, 3) = skew(Pgij + Pcij);
		b.block(3 * i, 0, 3, 1) = Pcij - Pgij;
	}
	

	
	//Computing Rotation:
	//Pcg_prime = Atsai.completeOrthogonalDecomposition().solve(b);
	Pcg_prime = (Atsai.transpose()*Atsai).inverse()*Atsai.transpose()*b;

	//Pcg_prime to rotation matrix
	Pcg = 2 * Pcg_prime / sqrt(1 + Pcg_prime.norm()*Pcg_prime.norm());
	Rcg = (1 - Pcg.norm()*Pcg.norm() / 2)*Matrix3d::Identity(3, 3) + 0.5*(Pcg*Pcg.transpose() + skew(Pcg)*sqrt(4 - Pcg.norm()*Pcg.norm()));

	//Compute Error
	err = Atsai*Pcg_prime - b;
	residus_TSAI_rotation = sqrt(err.dot(err) / (N - 1));

	//Solve Translation
	for (int i = 0; i < N - 1; i++)
	{
		C.block(3 * i, 0, 3, 3) = MatrixXd::Identity(3, 3) - Ra.block(3 * i, 0, 3, 3);
		d.block(3 * i, 0, 3, 1) = Ta.block(3 * i, 0, 3, 1) - Rcg*Tb.block(3 * i, 0, 3, 1);
	}
	//Tcg = C.completeOrthogonalDecomposition().solve(d);
	Tcg = (C.transpose()*C).inverse()*(C.transpose()*d);

	//Compute Error
	err = C*Tcg - d;
	residus_TSAI_translation = sqrt(err.dot(err) / (N - 1));

	std::cout << "Error Rotation: " << residus_TSAI_rotation << std::endl;
	std::cout << "Error Translation: " << residus_TSAI_translation << std::endl;

	//Create solution matrix
	Hcam2marker = MatrixXd::Identity(4, 4);
	Hcam2marker.block(0, 0, 3, 3) = Rcg;
	Hcam2marker.block(0, 3, 3, 1) = Tcg;

	return Hcam2marker;
}

/*
//EUCLIDEAN METHOD has a bug where it hangs when there is the same data pair in the collection
MatrixXd HandEye_calibrator::navy_calibration()
{
//Given the Hand to world and camera to target 4*N by 4 transforms stacked row wise,
//it solves the Camera to Hand transform assuming the target is stationary

//Ax = xB
for (int i = 0; i < N-1; i++)
{
//Stacking up A matrix getting Ra and Ta
A.block(4 * i, 0, 4, 4) = Hmarker2world.block(4 * (i + 1), 0, 4, 4).inverse()*Hmarker2world.block(4 * i, 0, 4, 4);

Ra.block(3 * i, 0, 3, 3) = A.block(4 * i, 0, 3, 3);
Ta.block(3 * i, 0, 3, 1) = A.block(4 * i, 3, 3, 1);

//Stacking up B matrix getting Tb
B.block(4 * i, 0, 4, 4) = Hgrid2cam.block(4 * (i + 1), 0, 4, 4)*Hgrid2cam.block(4 * i, 0, 4, 4).inverse();

Tb.block(3 * i, 0, 3, 1) = B.block(4 * i, 3, 3, 1);

//Matrix Logarithms
alpha = logMatrix(A.block(4 * i, 0, 4, 4)); //logmatrix Defined in KinematicsMaths.h
beta = logMatrix(B.block(4 * i, 0, 4, 4)); cout<<"alpha = "<<alpha<<endl;

M = M + beta*alpha.transpose();
}

//Solve Rotation
Rx = (M.transpose()*M).pow(-0.5)*M.transpose();

//Solve Translation
for (int i = 0; i < N - 1; i++)
{
	C.block(3 * i, 0, 3, 3) = MatrixXd::Identity(3, 3) - Ra.block(3 * i, 0, 3, 3);
	d.block(3 * i, 0, 3, 1) = Ta.block(3 * i, 0, 3, 1) - Rx*Tb.block(3 * i, 0, 3, 1);
}

Tx = (C.transpose()*C).inverse()*(C.transpose()*d);

//Create solution matrix
Hcam2marker = MatrixXd::Identity(4, 4);
Hcam2marker.block(0, 0, 3, 3) = Rx;
Hcam2marker.block(0, 3, 3, 1) = Tx;

return Hcam2marker;
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

//FUTURE FUNCTIONS

/*
//SAVE data to file
void HandEye_calibrator::Save_HandEye()
{
	//Take Hcam2marker convert to vector:
	vector<double> data;
	float value;
	for (int col = 0; col < 4; col++){
		for (int row = 0; row < 3; row++)
		{
			value = Hcam2marker(row,col);
			data.push_back(value); //{n,o,a,t}
		}
	}

	//Save vector to file
	vectorToFile('SnakeRaven_HandEye_marix.txt', data);
	cout<<"Saved as: SnakeRaven_HandEye_marix.txt"<<endl;
}

//Retrive data from file
void HandEye_calibrator::Retrieve_HandEye()
{
	//Take file convert to Hcam2marker:
	vector<double> data;
	data = fileToVector('SnakeRaven_HandEye_marix.txt');

	int i = 0;
	for (int col = 0; col < 4; col++){
		for (int row = 0; row < 3; row++)
		{
			Hcam2marker(row,col) = data->at(i);
			i++;
		}
	}

}
*/

/*
//Vector to file
void vectorToFile(char *name, vector<double> *a){
    FILE* fp = fopen(name, "w");
    for(int i=0;i<a->size();i++){
        for(int j=0;j<a->at(i).size();j++){
            fprintf(fp, "%f ", a->at(i).at(j));
        }
        fprintf(fp, "\n");
    }
    fclose(fp);
}

//File to vector
vector<double> fileToVector(const char *name)
{
    vector<double> result;
    std::ifstream input (name);
    std::string lineData;

    while(getline(input, lineData))
    {
        double d;
        std::vector<double> row;
        std::stringstream lineStream(lineData);

        while (lineStream >> d)
            row.push_back(d);

        result.push_back(row);
    }

    return result;
}
*/