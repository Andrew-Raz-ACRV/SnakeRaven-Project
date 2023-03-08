#include "Raven_Controller.h"

//**************************Teleoperation mode console_process********************//
/*
This file contains Raven_Controller Class function for the SnakeRaven Teleoperation part of the console process:

NOTE the other .cpp files for Raven_controller class:
- Raven_controller = where the class variables are initialised and the ROS communication occurs
- Console_process = where the robot state machine is defined
- Menu_screens = where functions for text display for the terminal are defined
- Keyboard_interactions = where functions for getting keyboard teleoperation commands are defined
- Raven_motion = where functions for robot motion are defined

author: Andrew Razjigaev 2023
*/

void Raven_Controller::teleop_process(int theKey, int counts)
{

	//Read current state:
	updateSnakejoints();					

	//For Different arm configs
	if(arm_config==rightonly)
	{
		//Update target and compute joint increment
		GreenSnake.Tend = GreenSnake.FK(); //Update Forward Kinematics
		Tend = GreenSnake.Tend;

		//Record Ttarget and Tend
		if(RECORDING){
			counts++;
			if(counts>(ROS_PUBLISH_RATE/record_freq)){
				record_to_csv();
				counts = 0;
			}
		}

		if (NO_TARGET_SET){
			Ttarget = GreenSnake.Tend;
			NO_TARGET_SET = false;
		}
		//cout << "key = " << theKey << endl;
		Ttarget = keyboard_teleop_map(theKey,Ttarget); //Ttarget update //GreenSnake.Tend

		dx = trans2dx(GreenSnake.Tend, Ttarget); //Update current Error
		
		cout << "Error = " << dx.norm() << endl;

		if (dx.norm() < tol_error) {
			cout << "Target Region Reached" << endl;
		}
		else {
			if (dx.norm() > dx_max) {
				dx = cap_mag(dx, dx_max); //Restrict delta size
			}
			//Update Jacobian, increment joint delta
			GreenSnake.J = GreenSnake.Jacobian(); //Update Jacobian

			//Weighted Damped Least Squares
			GreenSnake.q += dampedLeastSquares(GreenSnake.J, GreenSnake.q, GreenSnake.ql, GreenSnake.qu) * GreenSnake.W * dx;

			//Joint Limit check and saturation
			if (JointLimitcheck(GreenSnake.q, GreenSnake.ql, GreenSnake.qu)){
				cout << "\nWARNING: Workspace Limit Reached" << endl;
				//Saturate joint into jointspace
				GreenSnake.q = applyJointLimits(GreenSnake.q, GreenSnake.ql, GreenSnake.qu);
				//Move target back into workspace at that joint limit
				//GreenSnake.Tend = GreenSnake.FK();
				//Ttarget = GreenSnake.Tend;
			}
			//Update record of desired q joint value:
			GreenSnake.q_desired = GreenSnake.q;

			//Compute new Motor values
			GreenSnake.mv = GreenSnake.q2Motor_angles(); //Compute Motor values
			GoldSnake.mv = GoldSnake.q2Motor_angles(); //Compute Motor values

			updateRavenJointarray(); // send computed motor values

		}
	}
	else if(arm_config==leftonly)
	{
		//Update target and compute joint increment
		GoldSnake.Tend = GoldSnake.FK(); //Update Forward Kinematics
		Tend = GoldSnake.Tend;

		//Record Ttarget and Tend
		if(RECORDING){
			counts++;
			if(counts>(ROS_PUBLISH_RATE/record_freq)){
				record_to_csv();
				counts = 0;
			}
		}

		if (NO_TARGET_SET){
			Ttarget = GoldSnake.Tend;
			NO_TARGET_SET = false;
		}
		//cout << "key = " << theKey << endl;
		Ttarget = keyboard_teleop_map(theKey,Ttarget); //Ttarget update //GoldSnake.Tend

		dx = trans2dx(GoldSnake.Tend, Ttarget); //Update current Error
		
		cout << "Error = " << dx.norm() << endl;

		if (dx.norm() < tol_error) {
			cout << "Target Region Reached" << endl;
		}
		else {
			if (dx.norm() > dx_max) {
				dx = cap_mag(dx, dx_max); //Restrict delta size
			}
			//Update Jacobian, increment joint delta
			GoldSnake.J = GoldSnake.Jacobian(); //Update Jacobian

			//Weighted Damped Least Squares
			GoldSnake.q += dampedLeastSquares(GoldSnake.J, GoldSnake.q, GoldSnake.ql, GoldSnake.qu) * GoldSnake.W * dx;

			//Joint Limit check and saturation
			if (JointLimitcheck(GoldSnake.q, GoldSnake.ql, GoldSnake.qu)){
				cout << "\nWARNING: Workspace Limit Reached" << endl;
				//Saturate joint into jointspace
				GoldSnake.q = applyJointLimits(GoldSnake.q, GoldSnake.ql, GoldSnake.qu);
				//Move target back into workspace at that joint limit
				//GreenSnake.Tend = GreenSnake.FK();
				//Ttarget = GreenSnake.Tend;
			}
			//Update record of desired q joint value:
			GoldSnake.q_desired = GoldSnake.q;

			//Compute new Motor values
			GreenSnake.mv = GreenSnake.q2Motor_angles(); //Compute Motor values
			GoldSnake.mv = GoldSnake.q2Motor_angles(); //Compute Motor values

			updateRavenJointarray(); // send computed motor values

		}
	}
	else{
		//DUAL ARM TELEOPERATION
		//Update target and compute joint increment
		GreenSnake.Tend = GreenSnake.FK(); //Update Forward Kinematics
		Greenarm.Tend = GreenSnake.Tend;
		GoldSnake.Tend = GoldSnake.FK(); //Update Forward Kinematics
		Goldarm.Tend = GoldSnake.Tend;

		//Record Ttarget and Tend
		if(RECORDING){
			counts++;
			if(counts>(ROS_PUBLISH_RATE/record_freq)){
				record_to_csv();
				counts = 0;
			}
		}

		if (NO_TARGET_SET){
			Greenarm.Ttarget = GreenSnake.Tend;
			Goldarm.Ttarget = GoldSnake.Tend;
			NO_TARGET_SET = false;
		}
		//cout << "key = " << theKey << endl;
		Goldarm.Ttarget = keyboard_teleop_map(theKey,Goldarm.Ttarget); //Ttarget update //GreenSnake.Tend
		Greenarm.Ttarget = keyboard_teleop_map_numberpad(theKey,Greenarm.Ttarget);

		Goldarm.dx = trans2dx(GoldSnake.Tend, Goldarm.Ttarget);
		Greenarm.dx = trans2dx(GreenSnake.Tend, Greenarm.Ttarget); //Update current Error
		
		cout << "Error Left = " << Goldarm.dx.norm() << " Error right = " << Greenarm.dx.norm() << endl;

		//Restrict delta size
		if (Goldarm.dx.norm() > dx_max) {
			Goldarm.dx = cap_mag(Goldarm.dx, dx_max); 
		}
		if (Greenarm.dx.norm() > dx_max) {
			Greenarm.dx = cap_mag(Greenarm.dx, dx_max); 
		}		

		if((Greenarm.dx.norm() < tol_error) && (Goldarm.dx.norm() < tol_error))
		{
			//Both arms reached target or reset occured
			cout << "Target Region Reached" << endl;
		}	
		else if((Greenarm.dx.norm() < tol_error) && (Goldarm.dx.norm() >= tol_error))
		{
			//Green arm reached not gold so move gold

			//Update Jacobian, increment joint delta
			GoldSnake.J = GoldSnake.Jacobian(); //Update Jacobian

			//Weighted Damped Least Squares
			GoldSnake.q += dampedLeastSquares(GoldSnake.J, GoldSnake.q, GoldSnake.ql, GoldSnake.qu) * GoldSnake.W * Goldarm.dx;

			//Joint Limit check and saturation
			if (JointLimitcheck(GoldSnake.q, GoldSnake.ql, GoldSnake.qu)){
				cout << "\nWARNING: Workspace Limit Reached" << endl;
				//Saturate joint into jointspace
				GoldSnake.q = applyJointLimits(GoldSnake.q, GoldSnake.ql, GoldSnake.qu);
				//Move target back into workspace at that joint limit
				//GreenSnake.Tend = GreenSnake.FK();
				//Ttarget = GreenSnake.Tend;
			}
			//Update record of desired q joint value:
			GoldSnake.q_desired = GoldSnake.q;

			//Compute new Motor values
			GreenSnake.mv = GreenSnake.q2Motor_angles(); //Compute Motor values
			GoldSnake.mv = GoldSnake.q2Motor_angles(); //Compute Motor values

			updateRavenJointarray(); // send computed motor values
		}
		else if((Greenarm.dx.norm() >= tol_error) && (Goldarm.dx.norm() < tol_error))
		{
			//Gold arm reached not green so move green

			//Update Jacobian, increment joint delta
			GreenSnake.J = GreenSnake.Jacobian(); //Update Jacobian

			//Weighted Damped Least Squares
			GreenSnake.q += dampedLeastSquares(GreenSnake.J, GreenSnake.q, GreenSnake.ql, GreenSnake.qu) * GreenSnake.W * Greenarm.dx;

			//Joint Limit check and saturation
			if (JointLimitcheck(GreenSnake.q, GreenSnake.ql, GreenSnake.qu)){
				cout << "\nWARNING: Workspace Limit Reached" << endl;
				//Saturate joint into jointspace
				GreenSnake.q = applyJointLimits(GreenSnake.q, GreenSnake.ql, GreenSnake.qu);
				//Move target back into workspace at that joint limit
				//GreenSnake.Tend = GreenSnake.FK();
				//Ttarget = GreenSnake.Tend;
			}
			//Update record of desired q joint value:
			GreenSnake.q_desired = GreenSnake.q;

			//Compute new Motor values
			GreenSnake.mv = GreenSnake.q2Motor_angles(); //Compute Motor values
			GoldSnake.mv = GoldSnake.q2Motor_angles(); //Compute Motor values

			updateRavenJointarray(); // send computed motor values
		}
		else
		{
			//Move both arms
			//Update Jacobian, increment joint delta
			GreenSnake.J = GreenSnake.Jacobian(); //Update Jacobian
			GoldSnake.J = GoldSnake.Jacobian();

			//Weighted Damped Least Squares
			GreenSnake.q += dampedLeastSquares(GreenSnake.J, GreenSnake.q, GreenSnake.ql, GreenSnake.qu) * GreenSnake.W * Greenarm.dx;
			GoldSnake.q += dampedLeastSquares(GoldSnake.J, GoldSnake.q, GoldSnake.ql, GoldSnake.qu) * GoldSnake.W * Goldarm.dx;

			//Joint Limit check and saturation
			if (JointLimitcheck(GreenSnake.q, GreenSnake.ql, GreenSnake.qu)){
				cout << "\nWARNING: Workspace Limit Reached" << endl;
				//Saturate joint into jointspace
				GreenSnake.q = applyJointLimits(GreenSnake.q, GreenSnake.ql, GreenSnake.qu);
			}
			if (JointLimitcheck(GoldSnake.q, GoldSnake.ql, GoldSnake.qu)){
				cout << "\nWARNING: Workspace Limit Reached" << endl;
				//Saturate joint into jointspace
				GoldSnake.q = applyJointLimits(GoldSnake.q, GoldSnake.ql, GoldSnake.qu);
			}
			//Update record of desired q joint value:
			GreenSnake.q_desired = GreenSnake.q;
			GoldSnake.q_desired = GoldSnake.q;

			//Compute new Motor values
			GreenSnake.mv = GreenSnake.q2Motor_angles(); //Compute Motor values
			GoldSnake.mv = GoldSnake.q2Motor_angles(); //Compute Motor values

			updateRavenJointarray(); // send computed motor values
		}			

	}
}