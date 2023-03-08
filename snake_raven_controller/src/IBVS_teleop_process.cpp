#include "Raven_Controller.h"

//**************************IBVS teleoperation assistance mode console_process********************//
/*
This file contains Raven_Controller Class function for the SnakeRaven IBVS teleoperation assistance part of the console process:

NOTE the other .cpp files for Raven_controller class:
- Raven_controller = where the class variables are initialised and the ROS communication occurs
- Console_process = where the robot state machine is defined
- Menu_screens = where functions for text display for the terminal are defined
- Keyboard_interactions = where functions for getting keyboard teleoperation commands are defined
- Raven_motion = where functions for robot motion are defined

author: Andrew Razjigaev 2023
*/

void Raven_Controller::IBVS_teleop_process(int theKey, int counter)
{
	if(arm_config==rightonly)
	{
		//Make the robot respond faster to velocity in this mode:
		motor_maxspeed_rad1 = 0.001; //0.001
		motor_maxspeed_rad2 = 0.015; //0.015
		motor_maxspeed_mm = 0.00025;

		//Read current joint state:
		updateSnakejoints();

		//Update Forward kinematics
		GreenSnake.Tend = GreenSnake.FK();

		//Compute Current Camera Pose:
		Tcamera = GreenSnake.Tend * Tend2camera;

		//Depending on IBVS mode get Velocity from appropriate source:
		if (IBVS_active)
		{
			//Get position from teleop, orientation from imageprocessor node
			Vc = IBVS_semi_autonomous_teleop(theKey);
			cout << "Vc = " << Vc.transpose() << endl;
		}
		else{
			//Get Full Haptic feedback on camera velocity!
			Vc = IBVS_keyboard_teleop_map(theKey);
		}

		//Convert Velocity to desired camera transform
		Tcam_target = Tnorm(Tcamera * dx2trans(Vc)); 

		//Get desired endeffector pose from camera transform
		Ttarget = Tcam_target * Tcam2end;

		if(RECORDING_CAMERA&&IBVS_active){
			//Assume the freq is 1000Hz, lets make it to record_freq 1000/10 = 100cylces/record:
			//counts++;
			//if(counts>(ROS_PUBLISH_RATE/record_freq)){
			record_to_csv_camera_pose();
				//counts = 0;
			//}
		}

		//Inverse Kinematics increment from Ttarget to joint delta:
		IK_update(); 
		counter++;
	}
	else if(arm_config==leftonly)
	{
		//Make the robot respond faster to velocity in this mode:
		motor_maxspeed_rad1 = 0.001; //0.001
		motor_maxspeed_rad2 = 0.015; //0.015
		motor_maxspeed_mm = 0.00025;

		//Read current joint state:
		updateSnakejoints();

		//Update Forward kinematics
		GoldSnake.Tend = GoldSnake.FK();

		//Compute Current Camera Pose:
		Tcamera = GoldSnake.Tend * Tend2camera;

		//Depending on IBVS mode get Velocity from appropriate source:
		if (IBVS_active)
		{
			//Get position from teleop, orientation from imageprocessor node
			Vc = IBVS_semi_autonomous_teleop(theKey);
			cout << "Vc = " << Vc.transpose() << endl;
		}
		else{
			//Get Full Haptic feedback on camera velocity!
			Vc = IBVS_keyboard_teleop_map(theKey);
		}

		//Convert Velocity to desired camera transform
		Tcam_target = Tnorm(Tcamera * dx2trans(Vc)); 

		//Get desired endeffector pose from camera transform
		Ttarget = Tcam_target * Tcam2end;

		if(RECORDING_CAMERA&&IBVS_active){
			//Assume the freq is 1000Hz, lets make it to record_freq 1000/10 = 100cylces/record:
			//counts++;
			//if(counts>(ROS_PUBLISH_RATE/record_freq)){
			record_to_csv_camera_pose();
				//counts = 0;
			//}
		}

		//Inverse Kinematics increment from Ttarget to joint delta:
		IK_update(); 
		counter++;
	}
}