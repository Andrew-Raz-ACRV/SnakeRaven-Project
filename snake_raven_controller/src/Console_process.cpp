#include "Raven_Controller.h"

//**************************console_process********************//
/*
This file contains Raven_Controller Class function for the console process:

NOTE the other .cpp files for Raven_controller class:
- Raven_controller = where the class variables are initialised and the ROS communication occurs
- Console_process = where the robot state machine is defined
- Menu_screens = where functions for text display for the terminal are defined
- Keyboard_interactions = where functions for getting keyboard teleoperation commands are defined
- Raven_motion = where functions for robot motion are defined

NOTE: the console process also uses functions from files:
- Calibratedmotions_process
- 

author: Andrew Razjigaev 2023
*/

/**
*	\fn void *console_process(void)
*
* 	\brief this thread is dedicated to console io
*
* 	\param a pointer to void
*
*	\return void
*/
void* Raven_Controller::console_process(void)
{
	if(ros::isInitialized())
	{
		int theKey;
		int counts = 0;
		control_mode = 0;
		bool print_menu = true;
		bool waypoint_init = true;
		ros::Time time;
		time = time.now();
		state = idle;
		arm_config = dual;
		handeye_sublevel = reset_arm;

		while(ros::ok())
		{
			theKey = getKey();	

			if(!RECEIVED_FIRST){
				cout<<"Waiting for the first receive of raven_state..."<<endl;
			}
				
			//State machine	
			switch(state)
			{
				case idle:
				{
					print_menu = menu_words(print_menu); control_mode = 0;
					//check theKey to see if the state changes
					if(theKey=='0')
					{
						//change state to calibration
						//state = calibration;

						//First choose arm
						state = choose_arm;
						print_menu = true;
					}
					else if(theKey=='1')
					{
						//change state to joint control
						state = joint_control; control_mode = 1;
						print_menu = true;
					}
					else if(theKey=='2')
					{
						if(CALIBRATED&&RECEIVED_FIRST){
							//change state to teleop control
							state = snake_teleop; control_mode = 2;
							print_menu = true;
						}
						else{
							cout<<"\nCALIBRATION required before teleoperation"<<endl;
						}
					}
					else if(theKey=='3')
					{
						if(CALIBRATED&&RECEIVED_FIRST){
							state = reset; control_mode = 3;
							cout<<"\nResetting Robot moving to home configuration... "<<endl;
						}
						else{
							cout<<"\n Reset not necessary without calibration"<<endl;
						}
					}
					else if(theKey=='4')
					{
						if(CALIBRATED&&RECEIVED_FIRST){
							state = hand_eye_calibration; control_mode = 4;
							print_menu = true; counter = 0;
							cout<<"\nRESETTING ROBOT FOR HAND EYE CALIBRATION"<<endl;
							cout<<"\nRun cv_camera_node and imageprocessor"<<endl;
							cout<<"\nPut Aruco Marker in field of view:"<<endl;
						}
						else{
							cout<<"\n Calibrate Robot before doing hand-eye calibration"<<endl;
						}
					}
					else if(theKey=='5')
					{
						if(CALIBRATED&&RECEIVED_FIRST){
							//change state to IBVS+teleop control
							state = IBVS_orientation_control; control_mode = 5;
							print_menu = true; counter = 0;
						}
						else{
							cout<<"\nCALIBRATION required before IBVS+teleoperation"<<endl;
						}
					}
					else if(theKey=='6')
					{
						if(CALIBRATED&&RECEIVED_FIRST){
							//change state to IBVS+teleop control
							state = Waypoint_navigation; control_mode = 6;
							print_menu = true; counter = 0;
							waypoint_init = true;
						}
						else{
							cout<<"\nCALIBRATION required before Waypoint_navigation"<<endl;
						}						
					}

					//ADD NEW KEYBOARD INPUT FOR NEW MODES HERE

					break;
				}
				case choose_arm:
				{
					//Prompt user to choose arm
					print_menu = choose_arm_menu_words(print_menu);

					//Wait for user choice:
					if(theKey=='0')
					{
						//Right arm only Move left intial out of the way in calibration
						this->jpos_initial[SHOULDER_GOLD] = deg2rad(57);
						this->jpos_initial[ELBOW_GOLD] = deg2rad(-59);
						this->jpos_initial[Z_INS_GOLD] = 500; 
						this->jpos_initial[SHOULDER_GREEN] = deg2rad(-39.5967); //Move out of way deg2rad(-48);
						this->jpos_initial[ELBOW_GREEN] = deg2rad(-77.9160);    //Move out of way deg2rad(-36);
						this->jpos_initial[Z_INS_GREEN] = right_snaketool_length; //Move out of way 500;
						
						arm_config = rightonly;

						//change state to calibration
						state = calibration;
						print_menu = true;
					}
					else if(theKey=='1')
					{
						//Left arm only Move right intial out of the way in calibration
						this->jpos_initial[SHOULDER_GOLD] = deg2rad(39.5967); //Move out of way deg2rad(57);
						this->jpos_initial[ELBOW_GOLD] = deg2rad(-102.0840); //Move out of way deg2rad(-59);
						this->jpos_initial[Z_INS_GOLD] = left_snaketool_length; //snake_length tube length in mm;  //Move out of way deg2rad(500);
						this->jpos_initial[SHOULDER_GREEN] = deg2rad(-48);
						this->jpos_initial[ELBOW_GREEN] = deg2rad(-36);
						this->jpos_initial[Z_INS_GREEN] = 500; //Move out of way 500;

						arm_config = leftonly;

						//change state to calibration
						state = calibration;
						print_menu = true;
					}
					else if(theKey=='2')
					{
						//Dual arm
						//Default setting in Raven_Controller::init_sys()
						arm_config = dual;
						//change state to calibration
						state = calibration;
						print_menu = true;
					}
					break;
				}
				case calibration:
				{
					//Explain calibration procedure
					print_menu = calibration_menu_words(print_menu);

					//Move to initial configuration
					if (GO_HOME){
						if (move_2_home()==true){
							cout<<"\n\tFinished Move to home"<<endl;
							GO_HOME = false;
							MANUAL = true;
						}	
					}
					else{
						//Fine tune the robot manually, especially for tool placement
						MANUAL = manual_calibration_words(MANUAL);
						jointcontrol_keyboard_map(theKey);
					}

					//kill state
					if(theKey=='k')
					{
						//Go to calibrated motion mode
						//state = calibrated_motion; //DO Calibrated Motions state
						state = idle; // SKIP CALIBRATED MOTIONS
						checkpoint = 0;
						print_menu = true;

						//Get current jpos as offset in jpos to q conversion
						calibrate_snake_raven_state();
						CALIBRATED = true;
						cout<<"\nCalibration Set!"<<endl;
						if(state==calibrated_motion){
							cout<<"\nNow testing Calibration..."<<endl;
						}
						else{
							cout<<"\nCalibration Finished"<<endl;
						}
					}
					break;
				}
				case reset:
				{
					//Reset back to home:

					counter++;
					if (counter>5000){
						if(movebase_2_home()==true)
						{
							counter = 0;
							state = idle;
							print_menu = true;
							cout<<"\nReset complete."<<endl;
						}
					}
					else{
						//read current joint angles in calibrated form
						updateSnakejoints(); 

						//Set target joint angles:
						//Gold Arm
						//GoldSnake.q(0) = this->jpos_initial[SHOULDER_GOLD]; //deg2rad(39.5967);
						//GoldSnake.q(1) = this->jpos_initial[ELBOW_GOLD]; //deg2rad(-102.0840);
						GoldSnake.q(2) = 0;
						//Proximal
						GoldSnake.q(3) = 0;
						GoldSnake.q(4) = 0;
						//Distal
						if(GoldSnake.m==2)
						{
							GoldSnake.q(5) = 0;
							GoldSnake.q(6) = 0;
						}


						//Green Arm
						//GreenSnake.q(0) = this->jpos_initial[SHOULDER_GREEN];//deg2rad(-39.5967);
						//GreenSnake.q(1) = this->jpos_initial[ELBOW_GREEN]; //deg2rad(-77.9160);
						GreenSnake.q(2) = 0;
						//Proximal
						GreenSnake.q(3) = 0;
						GreenSnake.q(4) = 0;
						//Distal
						if(GreenSnake.m==2)
						{
							GreenSnake.q(5) = 0;
							GreenSnake.q(6) = 0;
						}

						//Compute new Motor values
						GreenSnake.mv = GreenSnake.q2Motor_angles(); //Compute Motor values
						GoldSnake.mv = GoldSnake.q2Motor_angles(); //Compute Motor values

						updateRavenJointarray(); // send computed motor values
					}

					break;

				}
				case joint_control:
				{
					//one time menu for joint control
					print_menu = joint_menu_words(print_menu);

					//Joint control state
					jointcontrol_keyboard_map(theKey);

					//kill state
					if(theKey=='k')
					{
						state = idle;
						print_menu = true;
						CALIBRATED = false; //must recalibrate if controlling joints individually
						cout<<"\nExiting mode..."<<endl;
					}
					break;
				}
				case calibrated_motion:
				{
					//THIS STATE WAS PART OF CALIBRATION FOR RIGHT ARM SNAKE - NO LONGER BEING USED
					//It basically moved each joint of the snake arm sequentially
					//when process is completed it outputs true and escapes mode
					if(calibrated_motions_process(counts))
					{
						state = idle;
						print_menu = true;
						cout<<"\nCalibrated motions complete..."<<endl;
						cout<<"\nExiting mode..."<<endl;
					}

					//kill state
					if(theKey=='k')
					{
						state = idle;
						print_menu = true;
						cout<<"\nExiting mode..."<<endl;
					}
					break;
				}
				case snake_teleop:
				{
					//one time menu for teleoperation
					if(arm_config==dual)
					{
						print_menu = teleopdual_menu_words(print_menu);
					}
					else
					{
						print_menu = teleop_menu_words(print_menu);
					}

					//Run teleop process
					teleop_process(theKey,counts);

					//kill state
					if(theKey=='k')
					{
						state = idle;
						print_menu = true;
						cout<<"\nExiting mode..."<<endl;
					}
					break;
				}
				case hand_eye_calibration:
				{
					if(arm_config!=dual)
					{
						//run Handeye process unless its completed
						if(handeye_process(theKey,counter))
						{
							handeye_sublevel = reset_arm;
							state = reset;
							print_menu = true;
							cout<<"\nExiting Hand Eye Calibration mode..."<<endl;
						}
					}
					else{
						cout<<"\nNot Implemented for dual arm configuration..."<<endl;
						cout<<"\nPlease only run hand-eye calibration one arm at a time"<<endl;
						state = idle;
						print_menu = true;
						cout<<"\nExiting mode..."<<endl;
					}

					//kill state
					if(theKey=='k')
					{
						state = idle;
						handeye_sublevel = reset_arm;
						print_menu = true;
						cout<<"\nHand Eye calibration Killed"<<endl;
						cout<<"\nExiting mode..."<<endl;
					}
					break;
				}
				case IBVS_orientation_control:
				{
					//THE BEST TELEOP MODE SO FAR!
					print_menu = IBVS_teleop_menu_words(print_menu);

					IBVS_teleop_process(theKey,counter);

					if(arm_config==dual){
						cout<<"\nNot Implemented for dual arm configuration..."<<endl;
						cout<<"\nPlease use the right arm for this mode"<<endl;

						//Revert back to default speed!
						motor_maxspeed_rad1 = 0.0001/2.0; //rad for larger joints //5.0
						motor_maxspeed_rad2 = 0.001/2.0; //rad for smaller joints
						motor_maxspeed_mm = 0.0001/15.0; //mm
						state = idle;
						print_menu = true;
						cout<<"\nExiting mode..."<<endl;
					}

					//kill state
					if(theKey=='k')
					{
						//Revert back to default speed!
						motor_maxspeed_rad1 = 0.0001/2.0; //rad for larger joints //5.0
						motor_maxspeed_rad2 = 0.001/2.0; //rad for smaller joints
						motor_maxspeed_mm = 0.0001/15.0; //mm
						//Go to idle state
						state = idle;
						print_menu = true;
						counter = 0;
						cout<<"\nExiting mode..."<<endl;
					}
					break;
				}
				case Waypoint_navigation:
				{
					print_menu = Waypoint_menu_words(print_menu);
					waypoint_init = WaypointInit(waypoint_init);

					if(Waypoint_navigation_process())
					{
						//Reset
						state = reset;
						print_menu = true;
						cout<<"\nCompleted Waypoint navigation task..."<<endl;						
					}

					//kill state
					if(theKey=='k')
					{
						//Go to idle state
						state = idle;
						print_menu = true;
						counter = 0;
						cout<<"\nExiting mode..."<<endl;
					}
					break;
				}
				//MAKE NEW CONTROL MODES DOWN HERE WITH A NEW CASE
			}

		}
		cout<<"console_process is shutdown."<<endl;

	}
	return 0;
}