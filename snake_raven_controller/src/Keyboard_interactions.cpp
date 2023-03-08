#include "Raven_Controller.h"

//**************************KEYBOARD INTERACTIONS********************//
/*
This file contains Raven_Controller Class functions related to keyboard interactions:

NOTE the other .cpp files for Raven_controller class:
- Raven_controller = where the class variables are initialised and the ROS communication occurs
- Console_process = where the robot state machine is defined
- Menu_screens = where functions for text display for the terminal are defined
- Keyboard_interactions = where functions for getting keyboard teleoperation commands are defined
- Raven_motion = where functions for robot motion are defined

author: Andrew Razjigaev 2023
*/


/**
*	\fn int getKey()
*
*	\brief gets keyboard character for switch case's of console_process()
*
* 	\param void
*
*	\return character int
*/
int Raven_Controller::getKey() 
{
    	int character;
    	struct termios orig_term_attr;
    	struct termios new_term_attr;

    	// set the terminal to raw mode 
    	tcgetattr(fileno(stdin), &orig_term_attr);
    	memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    	new_term_attr.c_lflag &= ~(ECHO|ICANON);
    	new_term_attr.c_cc[VTIME] = 0;
    	new_term_attr.c_cc[VMIN] = 0;
    	tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    	// read a character from the stdin stream without blocking 
    	//   returns EOF (-1) if no character is available 
    	character = fgetc(stdin);

   	// restore the original terminal attributes 
    	tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    	return character;
}

/**
*	\fn IBVS_keyboard_teleop_map Semi_autonomous
*
* 	\updates the target position based on keypresses
*
* 	\param int theKey from console.
*
*	\return Vc Matrix6d
*/
MatrixXd Raven_Controller::IBVS_semi_autonomous_teleop(int theKey)
{
	//Keyboard update step
	double delta = 2; //was 0.1mm
	//Default no velocity
	MatrixXd V = MatrixXd::Zero(6, 1);
	//int remainder = 1;

	//theKey is -1 when no key is pressed, setting default to be no target doesn't really work

	//MANUAL PART Increment Translational Velocities:
		switch(theKey)
		{
			//Teleoperation mapping
			case '1':
			{
				//Toggle IBVS_active!
				IBVS_active = false;
				control_mode = 5;
				cout << "Semi-autonomous: OFF" << endl;
				break;
			}
			case 'w':
			{
				//Z++
				//if (counter%remainder==0)
					V(2) = delta;
				//cout << "Target= " << endl << Ttarget << endl;
				break;
			}
			case 's':
			{
				//Z--
				//if (counter%remainder==0)
					V(2) = -delta;
				//cout << "Target= " << endl << Ttarget << endl;
				break;
			}
			case 'd':
			{
				//X++
				//if (counter%remainder==0)
					V(0) = delta;
				//cout << "Target= " << endl << Ttarget << endl;
				break;
			}
			case 'a':
			{
				//X--
				//if (counter%remainder==0)
					V(0) = -delta;
				//cout << "Target= " << endl << Ttarget << endl;
				break;
			}
			case 'q':
			{
				//Y
				//if (counter%remainder==0)
					V(1) = -delta;
				//cout << "Target= " << endl << Ttarget << endl;
				break;
			}
			case 'e':
			{
				//Y++
				//if (counter%remainder==0)
					V(1) = delta;
				//cout << "Target= " << endl << Ttarget << endl;
				break;
			}
			case 'z':
			{
				//Toggle Freeze
				IBVS_STOP = !IBVS_STOP;
				break;
			}
		}

	//Autonomous Part!
	//V.block(3, 0, 3, 1) = Wc;
	for (int i = 0; i < 3; i++)
	{
		if ((IBVS_STOP==false)&&(counter%5==0))
		{
			//Get Latest angular velocity
			V(3 + i) = Wc(i);
		}
	}


	return V;
}

/**
*	\fn IBVS_keyboard_teleop_map
*
* 	\updates the target position based on keypresses
*
* 	\param int theKey from console.
*
*	\return Vc Matrix6d
*/
MatrixXd Raven_Controller::IBVS_keyboard_teleop_map(int theKey)
{
	//, const MatrixXd& V //METHOD NEED TO BE REVISED 
	//Keyboard update step
	double delta = 2; //was 0.1mm
	double rot_delta = 0.06; //deg2rad(5); //rad was 0.2
	MatrixXd V = MatrixXd::Zero(6, 1);

	//theKey is -1 when no key is pressed, setting default to be no target doesn't really work

	//Increment Velocities:
	switch(theKey)
	{
		//Teleoperation mapping
		case '1':
		{
			//Toggle IBVS_active!
			IBVS_active = true;
			control_mode = 5.5; 
			cout << "Semi-autonomous: ON" << endl;
			break;
		}
		case 'w':
		{
			//Z++
			V(2) = delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 's':
		{
			//Z--
			V(2) = -delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 'd':
		{
			//X++
			V(0) = delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 'a':
		{
			//X--
			V(0) = -delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 'q':
		{
			//Y
			V(1) = -delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 'e':
		{
			//Y++
			V(1) = delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		//Orientation
		case 't':
		{
			V(3) = rot_delta; //rotx++
			break;
		}
		case 'g':
		{
			V(3) = -rot_delta; //rotx--
			break;
		}
		case 'f':
		{
			V(4) = -rot_delta; //roty--
			break;
		}
		case 'h':
		{
			V(4) = rot_delta; //roty++
			break;
		}
		//Freeze motion
		case 'z':
		{
			//freeze motion
			NO_TARGET_SET = true;
			break;
		}
	}
	return V;
}

/**
*	\fn keyboard_teleop_map
*
* 	\updates the target position based on keypresses
*
* 	\param int theKey from console, matrix T.
*
*	\return Ttarget Matrix4d
*/
MatrixXd Raven_Controller::keyboard_teleop_map(int theKey, const MatrixXd& T)
{
	//Keyboard update step
	double delta = 0.1; //was 0.1mm
	double rot_delta = deg2rad(0.2); //rad was 0.2
	double pan_delta = deg2rad(5); //rad
	Matrix4d Ttarget = T;

	//theKey is -1 when no key is pressed, setting default to be no target doesn't really work
	//GreenSnake.q(5) = 0;
	//GreenSnake.q(6) = 0;

	switch(theKey)
	{
		//Teleoperation mapping
		case 's':
		{
			//Y--
			Ttarget(1, 3) -= delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 'w':
		{
			//Y++
			Ttarget(1, 3) += delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 'd':
		{
			//X++
			Ttarget(0, 3) += delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 'a':
		{
			//X--
			Ttarget(0, 3) -= delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 'q':
		{
			//Z++
			Ttarget(2, 3) += delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 'e':
		{
			//Z--
			Ttarget(2, 3) -= delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 'z':
		{
			//freeze motion
			NO_TARGET_SET = true;
			break;
		}
		case 'f':
		{
			//roty++
			//Affine3d Tr(AngleAxisd(rot_delta, Vector3d::UnitY()));
			//Ttarget = Ttarget * Tr.matrix();
			//Alternative distal pan --
			//GreenSnake.q(5) -= pan_delta;
			//Ttarget = GreenSnake.FK();
			if(arm_config==rightonly)
			{
				GreenSnake.q(5) -= pan_delta;
				Ttarget = GreenSnake.FK();
			}
			else{
				//Gold arm or dual arm the same
				GoldSnake.q(5) -= pan_delta;
				Ttarget = GoldSnake.FK();
			}
			break;
		}
		case 'h':
		{
			//roty--
			//Affine3d Tr(AngleAxisd(-rot_delta, Vector3d::UnitY()));
			//Ttarget = Ttarget * Tr.matrix();
			//Alternative distal pan ++
			//GreenSnake.q(5) += pan_delta;
			//Ttarget = GreenSnake.FK();
			if(arm_config==rightonly)
			{
				GreenSnake.q(5) += pan_delta;
				Ttarget = GreenSnake.FK();
			}
			else{
				//Gold arm or dual arm the same
				GoldSnake.q(5) += pan_delta;
				Ttarget = GoldSnake.FK();
			}
			break;
		}
		case 't':
		{
			//rotx+
			//Affine3d Tr(AngleAxisd(rot_delta, Vector3d::UnitX()));
			//Ttarget = Ttarget * Tr.matrix();
			//Alternative distal tilt --
			//GreenSnake.q(6) -= pan_delta;
			//Ttarget = GreenSnake.FK();
			if(arm_config==rightonly)
			{
				GreenSnake.q(6) -= pan_delta;
				Ttarget = GreenSnake.FK();
			}
			else{
				//Gold arm or dual arm the same
				GoldSnake.q(6) -= pan_delta;
				Ttarget = GoldSnake.FK();
			}
			break;
		}
		case 'g':
		{
			//rotx-
			//Affine3d Tr(AngleAxisd(-rot_delta, Vector3d::UnitX()));
			//Ttarget = Ttarget * Tr.matrix();
			//Alternative distal tilt ++
			if(arm_config==rightonly)
			{
				GreenSnake.q(6) += pan_delta;
				Ttarget = GreenSnake.FK();
			}
			else{
				//Gold arm or dual arm the same
				GoldSnake.q(6) += pan_delta;
				Ttarget = GoldSnake.FK();
			}
			break;
		}
		case 'y':
		{
			//rotz+
			Affine3d Tr(AngleAxisd(rot_delta, Vector3d::UnitZ()));
			Ttarget = Ttarget * Tr.matrix();
			break;
		}		
		case 'r':
		{
			//rotz-
			Affine3d Tr(AngleAxisd(-rot_delta, Vector3d::UnitZ()));
			Ttarget = Ttarget * Tr.matrix();
			break;
		}

	}
	return Ttarget;
}

/**
*	\fn keyboard_teleop_map
*
* 	\updates the target position based on keypresses
*
* 	\param int theKey from console, matrix T.
*
*	\return Ttarget Matrix4d
*/
MatrixXd Raven_Controller::keyboard_teleop_map_numberpad(int theKey, const MatrixXd& T)
{
	//Keyboard update step
	double delta = 0.1; //was 0.1mm
	double rot_delta = deg2rad(0.2); //rad was 0.2
	double pan_delta = deg2rad(5); //rad
	Matrix4d Ttarget = T;

	//theKey is -1 when no key is pressed, setting default to be no target doesn't really work
	//GreenSnake.q(5) = 0;
	//GreenSnake.q(6) = 0;

	//This function is only used for the right arm in dual arm teleoperation

	switch(theKey)
	{
		//Teleoperation mapping
		case '2':
		{
			//Y--
			Ttarget(1, 3) -= delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case '5':
		{
			//Y++
			Ttarget(1, 3) += delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case '3':
		{
			//X++
			Ttarget(0, 3) += delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case '1':
		{
			//X--
			Ttarget(0, 3) -= delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case '4':
		{
			//Z++
			Ttarget(2, 3) += delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case '6':
		{
			//Z--
			Ttarget(2, 3) -= delta;
			//cout << "Target= " << endl << Ttarget << endl;
			break;
		}
		case 'z':
		{
			//freeze motion
			NO_TARGET_SET = true;
			break;
		}
		case '7':
		{
			//roty++
			//Affine3d Tr(AngleAxisd(rot_delta, Vector3d::UnitY()));
			//Ttarget = Ttarget * Tr.matrix();
			//Alternative distal pan --
			GreenSnake.q(5) -= pan_delta;
			Ttarget = GreenSnake.FK();
			break;
		}
		case '9':
		{
			//roty--
			//Affine3d Tr(AngleAxisd(-rot_delta, Vector3d::UnitY()));
			//Ttarget = Ttarget * Tr.matrix();
			//Alternative distal pan ++
			GreenSnake.q(5) += pan_delta;
			Ttarget = GreenSnake.FK();
			break;
		}
		case '/':
		{
			//rotx+
			//Affine3d Tr(AngleAxisd(rot_delta, Vector3d::UnitX()));
			//Ttarget = Ttarget * Tr.matrix();
			//Alternative distal tilt --
			GreenSnake.q(6) -= pan_delta;
			Ttarget = GreenSnake.FK();
			break;
		}
		case '8':
		{
			//rotx-
			//Affine3d Tr(AngleAxisd(-rot_delta, Vector3d::UnitX()));
			//Ttarget = Ttarget * Tr.matrix();
			//Alternative distal tilt ++
			GreenSnake.q(6) += pan_delta;
			Ttarget = GreenSnake.FK();
			break;
		}
		case '-':
		{
			//rotz+
			Affine3d Tr(AngleAxisd(rot_delta, Vector3d::UnitZ()));
			Ttarget = Ttarget * Tr.matrix();
			break;
		}		
		case '*':
		{
			//rotz-
			Affine3d Tr(AngleAxisd(-rot_delta, Vector3d::UnitZ()));
			Ttarget = Ttarget * Tr.matrix();
			break;
		}

	}
	return Ttarget;
}

/*
* \brief: based on a key, control each joint in the console in the following fashion
*
* \param int
*
* \return void
*/
void Raven_Controller::jointcontrol_keyboard_map(int theKey)
{
	float delta_rad = 0.01;
	float delta_mm = 0.001;
			switch(theKey)
			{
				//Gold ARM actions
				case '1':
				{
					//Up shoulder
					delta_joint[SHOULDER_GOLD]=delta_rad;
					cout<<"Moving SHOULDER_GOLD up"<<endl;
					break;
				}
				case 'q':
				{
					//Down shoulder
					delta_joint[SHOULDER_GOLD]=-delta_rad;
					cout<<"Moving SHOULDER_GOLD down"<<endl;
					break;
				}
				case '2':
				{
					//Up elbow
					delta_joint[ELBOW_GOLD]=delta_rad;
					cout<<"Moving ELBOW_GOLD up"<<endl;
					break;
				}
				case 'w':
				{
					//Down elbow
					delta_joint[ELBOW_GOLD]=-delta_rad;
					cout<<"Moving ELBOW_GOLD down"<<endl;
					break;
				}
				case '3':
				{
					//Up z_INS
					delta_joint[Z_INS_GOLD]=delta_mm;
					cout<<"Moving Z_INS_GOLD up"<<endl;
					break;
				}
				case 'e':
				{
					//Down Z_INS
					delta_joint[Z_INS_GOLD]=-delta_mm;
					cout<<"Moving Z_INS_GOLD down"<<endl;
					break;
				}
				case '4':
				{
					//Up TOOL_ROT_GOLD
					delta_joint[TOOL_ROT_GOLD]=delta_rad;
					cout<<"Moving TOOL_ROT_GOLD up"<<endl;
					break;
				}
				case 'r':
				{
					//down TOOL_ROT_GOLD
					delta_joint[TOOL_ROT_GOLD]=-delta_rad;
					cout<<"Moving TOOL_ROT_GOLD down"<<endl;
					break;
				}
				case '5':
				{
					//Up TOOL_ROT_GOLD
					delta_joint[WRIST_GOLD]=delta_rad;
					cout<<"Moving WRIST_GOLD up"<<endl;
					break;
				}
				case 't':
				{
					//down TOOL_ROT_GOLD
					delta_joint[WRIST_GOLD]=-delta_rad;
					cout<<"Moving WRIST_GOLD down"<<endl;
					break;
				}
				case '6':
				{
					//Up grasp 1
					delta_joint[GRASP1_GOLD]=delta_rad;
					cout<<"Moving GRASP1_GOLD up"<<endl;
					break;
				}
				case 'y':
				{
					//down grasp 1
					delta_joint[GRASP1_GOLD]=-delta_rad;
					cout<<"Moving GRASP1_GOLD down"<<endl;
					break;
				}
				case '7':
				{
					//Up grasp 2
					delta_joint[GRASP2_GOLD]=delta_rad;
					cout<<"Moving GRASP2_GOLD up"<<endl;
					break;
				}
				case 'u':
				{
					//down grasp 2
					delta_joint[GRASP2_GOLD]=-delta_rad;
					cout<<"Moving GRASP2_GOLD down"<<endl;
					break;
				}


				//GREEN ARM ACTIONS
				case 'a':
				{
					//Up shoulder
					delta_joint[SHOULDER_GREEN]=delta_rad;
					cout<<"Moving SHOULDER_GREEN up"<<endl;
					break;
				}
				case 'z':
				{
					//Down shoulder
					delta_joint[SHOULDER_GREEN]=-delta_rad;
					cout<<"Moving SHOULDER_GREEN down"<<endl;
					break;
				}
				case 's':
				{
					//Up elbow
					delta_joint[ELBOW_GREEN]=delta_rad;
					cout<<"Moving ELBOW_GREEN up"<<endl;
					break;
				}
				case 'x':
				{
					//Down elbow
					delta_joint[ELBOW_GREEN]=-delta_rad;
					cout<<"Moving ELBOW_GREEN down"<<endl;
					break;
				}
				case 'd':
				{
					//Up z_INS
					delta_joint[Z_INS_GREEN]=delta_mm;
					cout<<"Moving Z_INS_GREEN up"<<endl;
					break;
				}
				case 'c':
				{
					//Down Z_INS
					delta_joint[Z_INS_GREEN]=-delta_mm;
					cout<<"Moving Z_INS_GREEN down"<<endl;
					break;
				}
				case 'f':
				{
					//Up TOOL_ROT_GREEN
					delta_joint[TOOL_ROT_GREEN]=delta_rad;
					cout<<"Moving TOOL_ROT_GREEN up"<<endl;
					break;
				}
				case 'v':
				{
					//Down Z_INS
					delta_joint[TOOL_ROT_GREEN]=-delta_rad;
					cout<<"Moving TOOL_ROT_GREEN down"<<endl;
					break;
				}
				case 'g':
				{
					//Up WRIST
					delta_joint[WRIST_GREEN]=delta_rad;
					cout<<"Moving WRIST_GREEN up"<<endl;
					break;
				}
				case 'b':
				{
					//down wrist
					delta_joint[WRIST_GREEN]=-delta_rad;
					cout<<"Moving WRIST_GREEN down"<<endl;
					break;
				}
				case 'h':
				{
					//Up grasp 1
					delta_joint[GRASP1_GREEN]=delta_rad;
					cout<<"Moving GRASP1_GREEN up"<<endl;
					break;
				}
				case 'n':
				{
					//down grasp 1
					delta_joint[GRASP1_GREEN]=-delta_rad;
					cout<<"Moving GRASP1_GREEN down"<<endl;
					break;
				}
				case 'j':
				{
					//Up grasp 2
					delta_joint[GRASP2_GREEN]=delta_rad;
					cout<<"Moving GRASP2_GREEN up"<<endl;
					break;
				}
				case 'm':
				{
					//down grasp 2
					delta_joint[GRASP2_GREEN]=-delta_rad; 
					cout<<"Moving GRASP2_GREEN down"<<endl;
					break;
				}
	
			}
}


/*
							case 0:	
								for (int i = 3; i<GreenSnake.DOF; i++){GreenSnake.q(i) = 0;}
								break;
							case 1:
								for (int i = 3; i<GreenSnake.DOF; i++){GreenSnake.q(i) = 0;}
								GreenSnake.q(3) = 0.9*GreenSnake.qu(3);
								break;
							case 2:	
								for (int i = 3; i<GreenSnake.DOF; i++){GreenSnake.q(i) = 0;}
								break;
							case 3:
								for (int i = 3; i<GreenSnake.DOF; i++){GreenSnake.q(i) = 0;}
								GreenSnake.q(3) = 0.9*GreenSnake.ql(3);
								break;
							case 4:
								for (int i = 3; i<GreenSnake.DOF; i++){GreenSnake.q(i) = 0;}
								break;
							case 5:
								for (int i = 3; i<GreenSnake.DOF; i++){GreenSnake.q(i) = 0;}
								GreenSnake.q(4) = 0.9*GreenSnake.qu(4);
								break;
							case 6:
								for (int i = 3; i<GreenSnake.DOF; i++){GreenSnake.q(i) = 0;}
								break;
							case 7:
								for (int i = 3; i<GreenSnake.DOF; i++){GreenSnake.q(i) = 0;}
								GreenSnake.q(4) = 0.9*GreenSnake.ql(4);
								break;
							case 8:
								for (int i = 3; i<GreenSnake.DOF; i++){GreenSnake.q(i) = 0;}
								break;							
							case 9:
								state = idle;
								print_menu = true;
								cout<<"\nCalibrated motions complete..."<<endl;
								cout<<"\nExiting mode..."<<endl;
								break;
								*/