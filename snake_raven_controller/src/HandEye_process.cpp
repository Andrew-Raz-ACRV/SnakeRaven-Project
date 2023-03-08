#include "Raven_Controller.h"

//**************************Hand eye calibration process mode console_process********************//
/*
This file contains Raven_Controller Class function for the SnakeRaven hand eye calibration part of the console process:

NOTE the other .cpp files for Raven_controller class:
- Raven_controller = where the class variables are initialised and the ROS communication occurs
- Console_process = where the robot state machine is defined
- Menu_screens = where functions for text display for the terminal are defined
- Keyboard_interactions = where functions for getting keyboard teleoperation commands are defined
- Raven_motion = where functions for robot motion are defined

author: Andrew Razjigaev 2023
*/

bool Raven_Controller::handeye_process(int theKey, int counter)
{
	bool completion = false;
	//Sub level states
	switch(handeye_sublevel){
		case reset_arm:
		{
			//1. Reset Robot
			if (counter<=6000){
				Reset_Robot();
				counter++;
			}
			else{
				handeye_sublevel = collect; counter = 0; //print_menu = true;
				cout<<"\nReset complete."<<endl;
				cout<<"\nStarting Data Collection..."<<endl;
				cout<<"\nKeep Aruco Marker stationary!"<<endl;
				cout<<"\nCollected Data point:"<<endl;
			}
			break;
		}
		case collect:
		{
			//2. Collect Data
			Teleop_update(theKey);

			if (HandEye.ii<HandEye.N){
				//If the Aruco pose is coming in
				if (Aruco_transform != MatrixXd::Identity(4, 4)){
					//Increment data collection
					counter++;
					if ((counter>1000)&&(Aruco_transform != MatrixXd::Identity(4, 4))){
						//Take as valid data point
						updateSnakejoints();
						if(arm_config==rightonly)
						{
							Tend = GreenSnake.FK();
						}
						else if(arm_config==leftonly)
						{
							Tend = GoldSnake.FK();
						}
						HandEye.Hmarker2world.block(4 * HandEye.ii, 0, 4, 4) = Tend;//Twg;
						HandEye.Hgrid2cam.block(4 * HandEye.ii, 0, 4, 4) = Aruco_transform; //Tct;

						//Show data point:
						HandEye.ii++;
						cout<<HandEye.ii<<endl;
						cout<<"\nTwg = "<<endl<<Tend<<endl;
						cout<<"\nTct = "<<endl<<Aruco_transform<<endl;
						counter = 0;
					}
				}

			}
			else{
				handeye_sublevel = compute; counter = 0;
				cout<<"\nCollection complete."<<endl;
			}

			break;
		}
		case compute:
		{
			//3. When full, compute matrix
			cout<<"\nComputing Hand-eye Transformation:"<<endl;
			Tend2camera = HandEye.TSAIleastSquareCalibration();
			cout <<"Tend2camera = "<<endl;
			cout << Tend2camera << endl;
			HandEye.ii = 0;
			//Save

			//exit
			completion = true;
			break;
		}
	}

	return completion;
}