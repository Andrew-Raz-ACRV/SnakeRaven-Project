#include "Raven_Controller.h"

//**************************Menu Screens********************//
/*
This file contains Raven_Controller Class functions related to the menu screens:

NOTE the other .cpp files for Raven_controller class:
- Raven_controller = where the class variables are initialised and the ROS communication occurs
- Console_process = where the robot state machine is defined
- Menu_screens = where functions for text display for the terminal are defined
- Keyboard_interactions = where functions for getting keyboard teleoperation commands are defined
- Raven_motion = where functions for robot motion are defined

author: Andrew Razjigaev 2023
*/

/**
*	\fn void init_words() 
*
*	\brief show greeting words on console.
*
* 	\param void
*
*	\return void
*/
void Raven_Controller::init_words()  
{
	string start = "0";
	do
	{
		cout<<endl<<endl;
		cout<<"Welcome to the Snake Raven Controller"<<endl<<endl;
		cout<<"Please press \"Enter\" to start!";
		cin.clear();
		getline(std::cin,start);
	}while(start!="");

	cout<<"SnakeRaven Talker should be starting..."<<endl;
}

/**
*	\fn void menu_words() 
*
*	\brief show menu words on console.
*
* 	\param void
*
*	\return void
*/
bool Raven_Controller::menu_words(bool print_menu)  
{
	if(print_menu)
	{
		cout<<endl;
		cout<<"Snake Raven Controller Main Menu:"<<endl;
		cout<<"----------------------------------------------------"<<endl;
		cout<<"\t'0' : Set up Calibration"<<endl;
		cout<<"\t'1' : Joint Controller mode"<<endl;
		cout<<"\t'2' : Keyboard Teleoperation mode"<<endl;
		cout<<"\t'3' : Retract/reset arm go to Calibration pose"<<endl;
		cout<<"\t'4' : Hand-Eye Calibration With Aruco marker"<<endl;
		cout<<"\t'5' : IBVS assistance for orientation Teleoperation"<<endl;
		cout<<"\t'6' : Waypoint Navigation Task"<<endl;
		cout<<"\t'^C': Quit program"<<endl<<endl;
	}
	return false;
}

/**
*	\fn void menu_words() 
*
*	\brief show menu words on console.
*
* 	\param void
*
*	\return void
*/
bool Raven_Controller::calibration_menu_words(bool print_menu)  
{
	if(print_menu)
	{
		cout<<endl;
		cout<<"Calibration Begins..."<<endl;
		cout<<endl;
		cout<<"MOVING TO HOME CONFIGURATION..."<<endl;
		GO_HOME = true;
	}
	return false;
}

bool Raven_Controller::choose_arm_menu_words(bool print_menu)  
{
	if(print_menu)
	{
		cout<<endl;
		cout<<"Choose arm configuration:"<<endl;
		cout<<"----------------------------------------------------"<<endl;
		cout<<"\t'0' : Use only RIGHT Arm - Hide Left arm"<<endl;
		cout<<"\t'1' : Use only LEFT Arm - Hide right arm"<<endl;
		cout<<"\t'2' : Use DUAL ARMS"<<endl;
	}
	return false;
}

bool Raven_Controller::Waypoint_menu_words(bool print_menu)
{
	if(print_menu)
	{
		cout<<endl;
		cout<<"This mode makes the robot follow a trajectory of waypoints"<<endl;
		cout<<"----------------------------------------------------"<<endl;
	}
	return false;	
}

bool Raven_Controller::manual_calibration_words(bool print)  
{
	if(print)
	{
		cout<<endl;
		cout<<"Now check calibration manually..."<<endl;
		cout<<"Move joints manually to home configuration if it failed"<<endl;
		cout<<"Please put any custom instruments on robot if you haven't done so"<<endl;
		cout<<" -move the adaptor joints with keyboard"<<endl;
		cout<<endl;
		cout<<"SnakeRaven tool (on Gold & Green arms) should be completely perpendicular at RCM:"<<endl;
		cout<<"-----------------------------------------------------------------"<<endl;
		cout<<"\t'1 2 3 4 5 6 7' : Increase Gold arm DOF"<<endl;
		cout<<"\t'q w e r t y u' : Decrease Gold arm DOF"<<endl;
		cout<<"\t'a s d f g h j' : Increase Green arm DOF"<<endl;
		cout<<"\t'z x c v b n m' : Decrease Green arm DOF"<<endl;
		cout<<"\t'k' : finish manual calibration"<<endl;
		cout<<"\t'^C': Quit"<<endl<<endl;
	}
	return false;
}

/**
*	\fn void menu_words() 
*
*	\brief show menu words on console.
*
* 	\param void
*
*	\return void
*/
bool Raven_Controller::joint_menu_words(bool print_menu)  
{
	if(print_menu)
	{
		cout<<endl;
		cout<<"Snake Raven Joint controller Selection Menu:"<<endl;
		cout<<"----------------------------------------------------"<<endl;
		cout<<"\t'1 2 3 4 5 6 7' : Increase Gold arm DOF"<<endl;
		cout<<"\t'q w e r t y u' : Decrease Gold arm DOF"<<endl;
		cout<<"\t'a s d f g h j' : Increase Green arm DOF"<<endl;
		cout<<"\t'z x c v b n m' : Decrease Green arm DOF"<<endl;
		cout<<"\t'k' : return to main menu"<<endl;
		cout<<"\t'^C': Quit"<<endl<<endl;
	}
	return false;
}

/**
*	\fn void menu_words() 
*
*	\brief show menu words on console.
*
* 	\param void
*
*	\return void
*/
bool Raven_Controller::teleop_menu_words(bool print_menu)  
{
	if(print_menu)
	{
		cout<<endl;
		cout<<"Snake Raven Teleoperation Selection Menu:"<<endl;
		cout<<"Moves only a single arm not both:"<<endl;
		cout<<"----------------------------------------------------"<<endl;
		cout<<"\t'a d' : -/+ y axis"<<endl;
		cout<<"\t'w s' : +/- x axis"<<endl;
		cout<<"\t'q e' : +/- z axis"<<endl;
		cout<<"\t'f h' : +/- y rotation"<<endl;
		cout<<"\t'g t' : +/- x rotation"<<endl;
		cout<<"\t'r y' : +/- z rotation"<<endl;
		cout<<"\t'z' : reset target pose"<<endl;		
		cout<<"\t'k' : return to main menu"<<endl;
		cout<<"\t'^C': Quit"<<endl<<endl;
	}
	return false;
}

/**
*	\fn void menu_words() 
*
*	\brief show menu words on console.
*
* 	\param void
*
*	\return void
*/
bool Raven_Controller::teleopdual_menu_words(bool print_menu)  
{
	if(print_menu)
	{
		cout<<endl;
		cout<<"Snake Raven Teleoperation Selection Menu:"<<endl;
		cout<<"Moves both arms:"<<endl;
		cout<<"----------------------------------------------------"<<endl;
		cout<<"\t Left Arm: use e/q and aswd"<<endl;
		cout<<"\t'a d' : -/+ y axis"<<endl;
		cout<<"\t'w s' : +/- x axis"<<endl;
		cout<<"\t'q e' : +/- z axis"<<endl;
		cout<<"\t'f h' : +/- y rotation"<<endl;
		cout<<"\t'g t' : +/- x rotation"<<endl;
		cout<<"\t'r y' : +/- z rotation"<<endl;
		cout<<"\t Right Arm: use number pad"<<endl;
		cout<<"\t'1 3' : -/+ y axis"<<endl;
		cout<<"\t'5 2' : +/- x axis"<<endl;
		cout<<"\t'4 6' : +/- z axis"<<endl;
		cout<<"\t'7 9' : +/- y rotation"<<endl;
		cout<<"\t'8 /' : +/- x rotation"<<endl;
		cout<<"\t'* -' : +/- z rotation"<<endl;
		cout<<"\t'z' : reset target pose"<<endl;		
		cout<<"\t'k' : return to main menu"<<endl;
		cout<<"\t'^C': Quit"<<endl<<endl;
	}
	return false;
}

/**
*	\fn void menu_words() 
*
*	\brief show menu words on console.
*
* 	\param void
*
*	\return void
*/
bool Raven_Controller::IBVS_teleop_menu_words(bool print_menu)  
{
	if(print_menu)
	{
		cout<<endl;
		cout<<"Snake Raven IBVS guided Teleoperation Mapping:"<<endl;
		cout<<"Moves only a single arm not both:"<<endl;
		cout<<"----------------------------------------------------"<<endl;
		cout<<"\t'1'   : TOGGLE ACTIVATE/DEACTIVATE IBVS ASSIST"<<endl;
		cout<<"\t'w s' : +/- z axis FORWARD/ BACKWARD"<<endl;
		cout<<"\t'a d' : +/- x axis LEFT/ RIGHT"<<endl;
		cout<<"\t'q e' : +/- y axis UP/ DOWN"<<endl;
		cout<<"\t't g' : +/- x rotation"<<endl;
		cout<<"\t'h f' : +/- y rotation"<<endl;
		cout<<"\t'z'   : reset target pose"<<endl;		
		cout<<"\t'k'   : return to main menu"<<endl;
		cout<<"\t'^C'  : Quit"<<endl<<endl;
	}
	return false;
}

/**
*	\fn void final_words() 
*
*	\brief show goodbye words on console.
*
* 	\param void
*
*	\return void
*/
void Raven_Controller::final_words()  
{
	cout<<"Terminating the Snake Raven Controller." <<endl;
	cout<<"----------------------------------------------------"<<endl;
	cout<<"GoodBye!"<<endl<<endl<<endl;
}