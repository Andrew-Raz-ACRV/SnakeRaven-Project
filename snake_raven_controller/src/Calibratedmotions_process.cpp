#include "Raven_Controller.h"

//**************************Calibrated motions console_process********************//
/*
This file contains Raven_Controller Class function for the calibrated motions part of the console process:

NOTE the other .cpp files for Raven_controller class:
- Raven_controller = where the class variables are initialised and the ROS communication occurs
- Console_process = where the robot state machine is defined
- Menu_screens = where functions for text display for the terminal are defined
- Keyboard_interactions = where functions for getting keyboard teleoperation commands are defined
- Raven_motion = where functions for robot motion are defined

author: Andrew Razjigaev 2023
*/
bool Raven_Controller::calibrated_motions_process(int counts)
{
	bool completion = false;
		//For recording endeffector displacement ~works okay
	//Read current state Update Forward Kinematics
	//cout<<"stuck here"<<endl;
	updateSnakejoints();
	//cout<<"not stuck here"<<endl;
	GreenSnake.Tend = GreenSnake.FK();
	Tend = GreenSnake.Tend;
	if(RECORDING){
		//Assume the freq is 1000Hz, lets make it to record_freq 1000/10 = 100cylces/record:
		counts++;
		if(counts>(ROS_PUBLISH_RATE/record_freq)){
			record_to_csv();
			counts = 0;
		}
	}

	//For all checkpoints the raven joints are at initial state
	GreenSnake.q(0) = deg2rad(-39.5967);
	GreenSnake.q(1) = deg2rad(-77.9160);
	GreenSnake.q(2) = 0;

	//For all Desired Joint angles, make a ramp up and down factor
	
	if (counter<2000){
		//i.e. starts as 0 and increments  every iteration
		factor = factor + 0.00045; 
		//Will reach 0   +  0.00045*2000 = 0.9
	}
	else if(counter>4000){
		//Case when ramp down:
		factor = factor - 0.00045; 
		//Will reach 0.9  - 0.00045*2000 = 0.9-0.9 = 0
	}
	
	//factor = 0.9; //Original factor a constant

	//For single module:
	if(GreenSnake.m==1){
		//Move the snake tool to checkpoints to observe the calibration
		// the for rests all to 0, then it follows +pan -pan +tilt -tilt
		GreenSnake.q(3) = 0;
		GreenSnake.q(4) = 0;
		switch(checkpoint)
		{
			case 0:
				GreenSnake.q(3) = factor*GreenSnake.qu(3); //+pan
				break;
			case 1:
				GreenSnake.q(3) = factor*GreenSnake.ql(3); //-pan
				break;
			case 2:
				break; //break
			case 3:
				GreenSnake.q(4) = factor*GreenSnake.qu(4); //+tilt
				break;
			case 4:
				GreenSnake.q(4) = factor*GreenSnake.ql(4); //-tilt
				break;
			case 5:
				break; //break
			case 6:
				completion = true;
				//state = idle;
				//print_menu = true;
				//cout<<"\nCalibrated motions complete..."<<endl;
				//cout<<"\nExiting mode..."<<endl;
				break;
		}
	}
	else{
		//Double Module Checkpoint
		GreenSnake.q(3) = 0;
		GreenSnake.q(4) = 0;
		GreenSnake.q(5) = 0;
		GreenSnake.q(6) = 0;

		switch(checkpoint)
		{
			case 0:
				GreenSnake.q(5) = factor*GreenSnake.qu(5); //+p pan
				break;
			case 1:
				GreenSnake.q(5) = factor*GreenSnake.ql(5); //-p pan
				break;
			case 2:
				break; //break
			case 3:
				GreenSnake.q(6) = factor*GreenSnake.qu(6); //+p tilt
				break;
			case 4:
				GreenSnake.q(6) = factor*GreenSnake.ql(6); //-p tilt
				break;
			case 5:
				break; //break
			case 12: //Stop here
				GreenSnake.q(5) = factor*GreenSnake.qu(5); //+d pan
				break;
			case 7:
				GreenSnake.q(5) = factor*GreenSnake.ql(5); //-d pan
				break;
			case 8:
				break; //break
			case 9:
				GreenSnake.q(6) = factor*GreenSnake.qu(6); //+d tilt
				break;
			case 10:
				GreenSnake.q(6) = factor*GreenSnake.ql(6); //-d tilt
				break;
			case 11:
				break; //break
			case 6:
				completion = true;
				//state = idle;
				//print_menu = true;
				//cout<<"\nCalibrated motions complete..."<<endl;
				//cout<<"\nExiting mode..."<<endl;
				break;
		}
	}

	//Update Ttarget and record of desired q
	Ttarget = GreenSnake.FK();
	GreenSnake.q_desired = GreenSnake.q;

	//start message for the motion
	if (counter==0){
		cout<<"Checkpoint: "<<checkpoint<<endl;
		cout<<"Moving to configuration: "<<endl<<GreenSnake.q<<endl;
	}

	//Decide when to change checkpoints based on a counter of loops
	counter++;
	//cout<<counter<<endl;
	if (counter>6000){
		counter = 0;
		factor = 0;
		checkpoint++;
	}

	//Compute new Motor values
	GreenSnake.mv = GreenSnake.q2Motor_angles(); //Compute Motor values
	GoldSnake.mv = GoldSnake.q2Motor_angles(); //Compute Motor values

	updateRavenJointarray(); // send computed motor values

	return completion;
}