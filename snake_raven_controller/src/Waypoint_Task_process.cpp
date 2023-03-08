#include "Raven_Controller.h"

//**************************Waypoint navigation console_process********************//
/*
This file contains Raven_Controller Class function for the waypoint navigation part of the console process:

NOTE the other .cpp files for Raven_controller class:
- Raven_controller = where the class variables are initialised and the ROS communication occurs
- Console_process = where the robot state machine is defined
- Menu_screens = where functions for text display for the terminal are defined
- Keyboard_interactions = where functions for getting keyboard teleoperation commands are defined
- Raven_motion = where functions for robot motion are defined

author: Andrew Razjigaev 2023
*/
bool Raven_Controller::Waypoint_navigation_process()
{
	bool completion = false;
	float toleranceL = 2;
	float toleranceR = 7; 
	//Threshold (~mm) for how sensitive the controller should be to reaching the waypoint
	//If tolerance is too low, controller may not converge! 
	//With Snake endeffector attached, the controller struggles to converge so make tolerance higher ~7

	//Note: GotoPose is in Raven_motion.cpp and it performs the main control action

	//Waypoint navigation task
	if(arm_config==rightonly)
	{
		if(GotoPose(Goldarm.Tend, Greenarm.Ttarget, toleranceL, toleranceR))
		{
			//Update waypoint
			waypoint_id++;
			//Get next waypoints
			completion = GetWaypoints();
		}
	}
	else if(arm_config==leftonly)
	{
		if(GotoPose(Goldarm.Ttarget, Greenarm.Tend, toleranceL, toleranceR))
		{
			//Update waypoint
			waypoint_id++;
			//Get next waypoints
			completion = GetWaypoints();
		}		
	}
	else{
		//Dual arm
		if(GotoPose(Goldarm.Ttarget, Greenarm.Ttarget, toleranceL, toleranceR))
		{
			//Update waypoint
			waypoint_id++;
			//Get next waypoints
			completion = GetWaypoints();
		}
	}


	return completion;
}

//Function for determining the next waypoint
bool Raven_Controller::GetWaypoints()
{
	bool completion = false; 
	//Use if the waypoints are finite / otherwise leave as false for infinte motion

	//Note the Ttarget is relative to the world (i.e. robot base frame for each arm)
	Matrix4d TL = MatrixXd::Identity(4,4);
	Matrix4d TR = MatrixXd::Identity(4,4);
	switch(waypoint_id)
	{
		case 1:
		{
			//Example of relative transform
			cout << "Going to Way point 1" << endl;
			//World to Endeffector * endeffector to target
			TL(2, 3) += 100; //Move in endeffector z axis
			TR(2, 3) += 100; //Move in endeffector z axis
			UpdateTtarget(TL,TR);
			break;
		}
		case 2:
		{
			cout << "Going to Way point 2" << endl;

			//Base to RCM to point
			TL = Goldarm.Tend;
			TR = Greenarm.Tend;
			TL(0, 3) = LeftRCM(0, 3) - 10; //Position in x axis
			TL(1, 3) = LeftRCM(1, 3) + 0; //Position in y axis
			TL(2, 3) = LeftRCM(2, 3) - GoldSnake.Tool(2, 3) - 100; //Position in z axis
			TR(0, 3) = RightRCM(0, 3) + 10; //Position in x axis
			TR(1, 3) = RightRCM(1, 3) + 0; //Position in y axis
			TR(2, 3) = RightRCM(2, 3) - GreenSnake.Tool(2, 3) - 100; //Position in z axis
			UpdateTtargetGlobal(TL, TR);

			cout << "Gold:" << endl;
			cout << Goldarm.Ttarget(0,3) << endl;
			cout << Goldarm.Ttarget(1,3) << endl;
			cout << Goldarm.Ttarget(2,3) << endl;
			cout << "Green:" << endl;
			cout << Greenarm.Ttarget(0,3) << endl;
			cout << Greenarm.Ttarget(1,3) << endl;
			cout << Greenarm.Ttarget(2,3) << endl;

			break;
		}
		case 3:
		{
			cout << "Going to Way point 3" << endl;
			
			//Base to RCM to point
			TL = Goldarm.Tend;
			TR = Greenarm.Tend;
			TL(0, 3) = LeftRCM(0, 3) - 10; //Position in x axis
			TL(1, 3) = LeftRCM(1, 3) + 10; //Position in y axis
			TL(2, 3) = LeftRCM(2, 3) - GoldSnake.Tool(2, 3) - 100; //Position in z axis
			TR(0, 3) = RightRCM(0, 3) + 10; //Position in x axis
			TR(1, 3) = RightRCM(1, 3) + 10; //Position in y axis
			TR(2, 3) = RightRCM(2, 3) - GreenSnake.Tool(2, 3) - 100; //Position in z axis
			UpdateTtargetGlobal(TL, TR);

			cout << "Gold:" << endl;
			cout << Goldarm.Ttarget(0,3) << endl;
			cout << Goldarm.Ttarget(1,3) << endl;
			cout << Goldarm.Ttarget(2,3) << endl;
			cout << "Green:" << endl;
			cout << Greenarm.Ttarget(0,3) << endl;
			cout << Greenarm.Ttarget(1,3) << endl;
			cout << Greenarm.Ttarget(2,3) << endl;

			break;
		}
		case 4:
		{
			cout << "Going to Way point 4" << endl;

			//Base to RCM to point
			TL = Goldarm.Tend;
			TR = Greenarm.Tend;
			TL(0, 3) = LeftRCM(0, 3) + 10; //Position in x axis
			TL(1, 3) = LeftRCM(1, 3) + 10; //Position in y axis
			TL(2, 3) = LeftRCM(2, 3) - GoldSnake.Tool(2, 3) - 100; //Position in z axis
			TR(0, 3) = RightRCM(0, 3) - 10; //Position in x axis
			TR(1, 3) = RightRCM(1, 3) + 10; //Position in y axis
			TR(2, 3) = RightRCM(2, 3) - GreenSnake.Tool(2, 3) - 100; //Position in z axis
			UpdateTtargetGlobal(TL, TR);
			
			cout << "Gold:" << endl;
			cout << Goldarm.Ttarget(0,3) << endl;
			cout << Goldarm.Ttarget(1,3) << endl;
			cout << Goldarm.Ttarget(2,3) << endl;
			cout << "Green:" << endl;
			cout << Greenarm.Ttarget(0,3) << endl;
			cout << Greenarm.Ttarget(1,3) << endl;
			cout << Greenarm.Ttarget(2,3) << endl;

			break;
		}
		case 5:
		{
			cout << "Going to Way point 5" << endl;

			//Base to RCM to point
			TL = Goldarm.Tend;
			TR = Greenarm.Tend;
			TL(0, 3) = LeftRCM(0, 3) + 10; //Position in x axis
			TL(1, 3) = LeftRCM(1, 3) - 10; //Position in y axis
			TL(2, 3) = LeftRCM(2, 3) - GoldSnake.Tool(2, 3) - 100; //Position in z axis
			TR(0, 3) = RightRCM(0, 3) - 10; //Position in x axis
			TR(1, 3) = RightRCM(1, 3) - 10; //Position in y axis
			TR(2, 3) = RightRCM(2, 3) - GreenSnake.Tool(2, 3) - 100; //Position in z axis
			UpdateTtargetGlobal(TL, TR);
			
			cout << "Gold:" << endl;
			cout << Goldarm.Ttarget(0,3) << endl;
			cout << Goldarm.Ttarget(1,3) << endl;
			cout << Goldarm.Ttarget(2,3) << endl;
			cout << "Green:" << endl;
			cout << Greenarm.Ttarget(0,3) << endl;
			cout << Greenarm.Ttarget(1,3) << endl;
			cout << Greenarm.Ttarget(2,3) << endl;

			break;
		}
		case 6:
		{
			cout << "Going to Way point 6" << endl;

			//Base to RCM to point
			TL = Goldarm.Tend;
			TR = Greenarm.Tend;
			TL(0, 3) = LeftRCM(0, 3) - 10; //Position in x axis
			TL(1, 3) = LeftRCM(1, 3) - 10; //Position in y axis
			TL(2, 3) = LeftRCM(2, 3) - GoldSnake.Tool(2, 3) - 100; //Position in z axis
			TR(0, 3) = RightRCM(0, 3) + 10; //Position in x axis
			TR(1, 3) = RightRCM(1, 3) - 10; //Position in y axis
			TR(2, 3) = RightRCM(2, 3) - GreenSnake.Tool(2, 3) - 100; //Position in z axis
			UpdateTtargetGlobal(TL, TR);
			
			cout << "Gold:" << endl;
			cout << Goldarm.Ttarget(0,3) << endl;
			cout << Goldarm.Ttarget(1,3) << endl;
			cout << Goldarm.Ttarget(2,3) << endl;
			cout << "Green:" << endl;
			cout << Greenarm.Ttarget(0,3) << endl;
			cout << Greenarm.Ttarget(1,3) << endl;
			cout << Greenarm.Ttarget(2,3) << endl;

			break;
		}
		case 7:
		{
			cout << "Going to Way point 7" << endl;

			//Base to RCM to point
			TL = Goldarm.Tend;
			TR = Greenarm.Tend;
			TL(0, 3) = LeftRCM(0, 3) - 10; //Position in x axis
			TL(1, 3) = LeftRCM(1, 3) + 10; //Position in y axis
			TL(2, 3) = LeftRCM(2, 3) - GoldSnake.Tool(2, 3) - 100; //Position in z axis
			TR(0, 3) = RightRCM(0, 3) + 10; //Position in x axis
			TR(1, 3) = RightRCM(1, 3) + 10; //Position in y axis
			TR(2, 3) = RightRCM(2, 3) - GreenSnake.Tool(2, 3) - 100; //Position in z axis
			UpdateTtargetGlobal(TL, TR);
			
			cout << "Gold:" << endl;
			cout << Goldarm.Ttarget(0,3) << endl;
			cout << Goldarm.Ttarget(1,3) << endl;
			cout << Goldarm.Ttarget(2,3) << endl;
			cout << "Green:" << endl;
			cout << Greenarm.Ttarget(0,3) << endl;
			cout << Greenarm.Ttarget(1,3) << endl;
			cout << Greenarm.Ttarget(2,3) << endl;

			//Repeat
			waypoint_id = 3;
			break;
		}
		default:
		{
			completion = true;
		}
	}
	return completion;
}

//Update relative to endeffector pose
void Raven_Controller::UpdateTtarget(const MatrixXd& TL, const MatrixXd& TR)
{
	if(arm_config==rightonly)
	{
		Greenarm.Ttarget = Greenarm.Tend * TR;
	}
	else if(arm_config==leftonly)
	{
		Goldarm.Ttarget = Goldarm.Tend * TL;
	}
	else{
		Goldarm.Ttarget = Goldarm.Tend * TL;
		Greenarm.Ttarget = Greenarm.Tend * TR;
	}
}

//Update relative to the robot base:
void Raven_Controller::UpdateTtargetGlobal(const MatrixXd& TL, const MatrixXd& TR)
{
	if(arm_config==rightonly)
	{
		Greenarm.Ttarget = TR;
	}
	else if(arm_config==leftonly)
	{
		Goldarm.Ttarget = TL;
	}
	else{
		Goldarm.Ttarget = TL;
		Greenarm.Ttarget = TR;
	}
}

//Function for setting initial waypoint:
bool Raven_Controller::WaypointInit(bool waypoint_init)
{
	if(waypoint_init)
	{
		//Set initial waypoint as current pose:
		waypoint_id = 0;
	
		//Calculate forward kinematics:
		GreenSnake.Tend = GreenSnake.FK(); //Update Forward Kinematics
		Greenarm.Tend = GreenSnake.Tend;
		GoldSnake.Tend = GoldSnake.FK(); //Update Forward Kinematics
		Goldarm.Tend = GoldSnake.Tend;

		//Set target as Tend
		Goldarm.Ttarget = Goldarm.Tend;
		Greenarm.Ttarget = Greenarm.Tend;
	}
	return false;
}