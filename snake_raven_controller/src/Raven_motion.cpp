#include "Raven_Controller.h"

//**************************Raven_Motion********************//
/*
This file contains Raven_Controller Class functions related to motion commands:

NOTE the other .cpp files for Raven_controller class:
- Raven_controller = where the class variables are initialised and the ROS communication occurs
- Console_process = where the robot state machine is defined
- Menu_screens = where functions for text display for the terminal are defined
- Keyboard_interactions = where functions for getting keyboard teleoperation commands are defined
- Raven_motion = where functions for robot motion are defined

author: Andrew Razjigaev 2023
*/

//SEND RESET INCREMENT COMMAND
void Raven_Controller::Reset_Robot()
{
	//Reset back to home:
	updateSnakejoints();

	//Set target joint angles:
	//Gold Arm
	GoldSnake.q(0) = this->jpos_initial[SHOULDER_GOLD]; //deg2rad(39.5967);
	GoldSnake.q(1) = this->jpos_initial[ELBOW_GOLD]; //deg2rad(-102.0840);
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
	GreenSnake.q(0) = this->jpos_initial[SHOULDER_GREEN];//deg2rad(-39.5967);
	GreenSnake.q(1) = this->jpos_initial[ELBOW_GREEN]; //deg2rad(-77.9160);
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

// Function for moving the robot to a given waypoint
bool Raven_Controller::GotoPose(const MatrixXd& TL, const MatrixXd& TR, float tolerance_errorL, float tolerance_errorR)
{
	//Run algorithm to move robot to pose
	updateSnakejoints();

	//Calculate forward kinematics:
	GreenSnake.Tend = GreenSnake.FK(); //Update Forward Kinematics
	Greenarm.Tend = GreenSnake.Tend;
	GoldSnake.Tend = GoldSnake.FK(); //Update Forward Kinematics
	Goldarm.Tend = GoldSnake.Tend;

	//Calculate error to target
	Goldarm.dx = trans2dx(GoldSnake.Tend, TL);
	Greenarm.dx = trans2dx(GreenSnake.Tend, TR); 

	//Useful debug message of progress towards target
	//cout << "Error Left = " << Goldarm.dx.norm() << " Error right = " << Greenarm.dx.norm() << endl;

	//Determine if waypoint is reached to reasonable accuracy
	if((Greenarm.dx.norm() < tolerance_errorR) && (Goldarm.dx.norm() < tolerance_errorL))
	{
		//Both arms reached target go to pose complete
		return true;
	}	
	else if((Greenarm.dx.norm() < tolerance_errorR) && (Goldarm.dx.norm() >= tolerance_errorL))
	{
		//Green arm reached not gold so move gold
		if (Goldarm.dx.norm() > dx_max) {
			Goldarm.dx = cap_mag(Goldarm.dx, dx_max); 
		}

		//Update Jacobian, increment joint delta
		GoldSnake.J = GoldSnake.Jacobian(); //Update Jacobian

		//Weighted Damped Least Squares
		GoldSnake.q += dampedLeastSquares(GoldSnake.J, GoldSnake.q, GoldSnake.ql, GoldSnake.qu) * GoldSnake.W * Goldarm.dx;

		//Joint Limit check and saturation
		if (JointLimitcheck(GoldSnake.q, GoldSnake.ql, GoldSnake.qu)){
			//cout << "\nWARNING: Workspace Limit Reached" << endl;
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

		return false;
	}
	else if((Greenarm.dx.norm() >= tolerance_errorR) && (Goldarm.dx.norm() < tolerance_errorL))
	{
		//Gold arm reached not green so move green
		if (Greenarm.dx.norm() > dx_max) {
			Greenarm.dx = cap_mag(Greenarm.dx, dx_max); 
		}

		//Update Jacobian, increment joint delta
		GreenSnake.J = GreenSnake.Jacobian(); //Update Jacobian

		//Weighted Damped Least Squares
		GreenSnake.q += dampedLeastSquares(GreenSnake.J, GreenSnake.q, GreenSnake.ql, GreenSnake.qu) * GreenSnake.W * Greenarm.dx;

		//Joint Limit check and saturation
		if (JointLimitcheck(GreenSnake.q, GreenSnake.ql, GreenSnake.qu)){
			//cout << "\nWARNING: Workspace Limit Reached" << endl;
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

		return false;
	}
	else
	{
		//Move both arms
		//Restrict delta size
		if (Goldarm.dx.norm() > dx_max) {
			Goldarm.dx = cap_mag(Goldarm.dx, dx_max); 
		}
		if (Greenarm.dx.norm() > dx_max) {
			Greenarm.dx = cap_mag(Greenarm.dx, dx_max); 
		}
		//Update Jacobian, increment joint delta
		GreenSnake.J = GreenSnake.Jacobian(); //Update Jacobian
		GoldSnake.J = GoldSnake.Jacobian();

		//Weighted Damped Least Squares
		GreenSnake.q += dampedLeastSquares(GreenSnake.J, GreenSnake.q, GreenSnake.ql, GreenSnake.qu) * GreenSnake.W * Greenarm.dx;
		GoldSnake.q += dampedLeastSquares(GoldSnake.J, GoldSnake.q, GoldSnake.ql, GoldSnake.qu) * GoldSnake.W * Goldarm.dx;

		//Joint Limit check and saturation
		if (JointLimitcheck(GreenSnake.q, GreenSnake.ql, GreenSnake.qu)){
			//cout << "\nWARNING: Workspace Limit Reached" << endl;
			//Saturate joint into jointspace
			GreenSnake.q = applyJointLimits(GreenSnake.q, GreenSnake.ql, GreenSnake.qu);
		}
		if (JointLimitcheck(GoldSnake.q, GoldSnake.ql, GoldSnake.qu)){
			//cout << "\nWARNING: Workspace Limit Reached" << endl;
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

		return false;
	}
}

//From using Ttarget to output, this does the inverse kinematics steps for it
void Raven_Controller::IK_update()
{
	if(arm_config==rightonly)
	{
		dx = trans2dx(GreenSnake.Tend, Ttarget); //Update current Error
						
		//cout << "Error = " << dx.norm() << endl;

		if (dx.norm() < tol_error) {
			//cout << "Target Region Reached" << endl;
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
		dx = trans2dx(GoldSnake.Tend, Ttarget); //Update current Error
						
		//cout << "Error = " << dx.norm() << endl;

		if (dx.norm() < tol_error) {
			//cout << "Target Region Reached" << endl;
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
	else
	{
		//Dual arm
	}
}

//SEND TELEOP INCREMENT COMMAND
void Raven_Controller::Teleop_update(int theKey)
{
	//Read current state:
	updateSnakejoints();

	if(arm_config==rightonly)
	{
		//Update target and compute joint increment
		GreenSnake.Tend = GreenSnake.FK(); //Update Forward Kinematics
		Tend = GreenSnake.Tend;

		if (NO_TARGET_SET){
			Ttarget = GreenSnake.Tend;
			NO_TARGET_SET = false;
		}

		Ttarget = keyboard_teleop_map(theKey,Ttarget); //Ttarget update //GreenSnake.Tend

		dx = trans2dx(GreenSnake.Tend, Ttarget); //Update current Error
						
		//cout << "Error = " << dx.norm() << endl;

		if (dx.norm() < tol_error) {
			//cout << "Target Region Reached" << endl;
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

		if (NO_TARGET_SET){
			Ttarget = GoldSnake.Tend;
			NO_TARGET_SET = false;
		}

		Ttarget = keyboard_teleop_map(theKey,Ttarget); //Ttarget update //GreenSnake.Tend

		dx = trans2dx(GoldSnake.Tend, Ttarget); //Update current Error
						
		//cout << "Error = " << dx.norm() << endl;

		if (dx.norm() < tol_error) {
			//cout << "Target Region Reached" << endl;
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
	else
	{
		//Dual arm
	}
}

/*
/ Brief: determines the offsets from home position set in the calibration mode
/
*/
void Raven_Controller::calibrate_snake_raven_state()
{
	//We know the current raven_state is equal to the initial q 
	//when this function is called in calibration
	//
	// CALIBRATION EQUATIONS
	// mv = jpos2mv * jpos + offset
	// mv = jpos / mv2jpos + offset
	//
	// offset = mv - jpos / mv2jpos
	//
	// jpos = mv2jpos * (mv - offset)

	//Joint values in snakeraven class reset to initial
	GreenSnake.reset_q();
	GoldSnake.reset_q();

	//Reset target 
	NO_TARGET_SET = true;	

	//Compute offset for Gold Snake
	for (int i = SHOULDER_GOLD; i<(SHOULDER_GOLD+GoldSnake.DOF); i++){
		calibrate_offset[i] = GoldSnake.mv(i) - jpos_current[i]/mv2jpos_rate[i];
	}

	//Compute offset for Green Snake
	for (int i = SHOULDER_GREEN; i<(SHOULDER_GREEN+GreenSnake.DOF); i++){
		calibrate_offset[i] = GreenSnake.mv(i-SHOULDER_GREEN) - jpos_current[i]/mv2jpos_rate[i];
	}	
}


//
// Move all joints to jpos_initial
//
bool Raven_Controller::move_2_home()
{
	bool reached = true; //assume it is there
	float error;
	//Check the joints and move
	for (int i = 0; i<14; i++){
		//check if the robot is there or not
		error = jpos_initial[i] - jpos_current[i];
		//send delta
		if ((i==Z_INS_GOLD)||(i==Z_INS_GREEN)){
			//prismatic
			if (reached && (fabs(error)<1)){
				reached = true;
			}
			else if (fabs(error)<1){
				delta_joint[i] = 0; //filters the jittering
			}
			else{
				reached = false;
				delta_joint[i] = saturate_round(error, motor_maxspeed_mm);
			}
		}
		else{
			//revolute
			if (reached && (fabs(error)<deg2rad(0.5))){
				reached = true;
			}
			else if (fabs(error)<deg2rad(0.5)){
				delta_joint[i] = 0; //filters the jittering
			}
			else{
				reached = false;
				delta_joint[i] = saturate_round(error, motor_maxspeed_rad1);
			}
		}
	}

	return reached;
}

//
// Move macro arm (shoulder,elbow,insertion) joints to jpos_initial
//
bool Raven_Controller::movebase_2_home()
{
	bool reached = true; //assume it is there
	float error;
	//Check the joints and move
	for (int i = 0; i<14; i++){
		//Logic to only move 0,1,2,7,8,9
		if((i<3)||((i>6)&&(i<10)))
		{
			//check if the robot is there or not
			error = jpos_initial[i] - jpos_current[i];
			//send delta
			if ((i==Z_INS_GOLD)||(i==Z_INS_GREEN)){
				//prismatic
				if (reached && (fabs(error)<1)){
					reached = true;
				}
				else if (fabs(error)<1){
					delta_joint[i] = 0; //filters the jittering
				}
				else{
					reached = false;
					delta_joint[i] = saturate_round(error, motor_maxspeed_mm);
				}
			}
			else{
				//revolute
				if (reached && (fabs(error)<deg2rad(0.5))){
					reached = true;
				}
				else if (fabs(error)<deg2rad(0.5)){
					delta_joint[i] = 0; //filters the jittering
				}
				else{
					reached = false;
					delta_joint[i] = saturate_round(error, motor_maxspeed_rad1);
				}
			}
		}
	}

	return reached;
}


/*
* \brief: puts the raven_state data into the two snakeraven class objects
* 
* \param void
*
* \return void
*/
void Raven_Controller::updateSnakejoints()
{
	//Use Calibration mv = jpos / mv2jpos + offset
	//Only the first 3 DOF because there is no reverse 
	//reading for tendon displacement to pan tilt angles
	for (int i = SHOULDER_GOLD; i<(SHOULDER_GOLD+GoldSnake.DOF); i++){
		//Left arm
		GoldSnake.mv_pre(i) = jpos_current[i] / mv2jpos_rate[i] + calibrate_offset[i];
	}

	for (int i = SHOULDER_GREEN; i<(SHOULDER_GREEN+GreenSnake.DOF); i++){
		//Right arm
		GreenSnake.mv_pre(i-SHOULDER_GREEN) = jpos_current[i] / mv2jpos_rate[i] + calibrate_offset[i];
	}

	//Convert the motor angles into pan and tilt joints
	GoldSnake.q = GoldSnake.Motor_angles2q();
	GreenSnake.q = GreenSnake.Motor_angles2q();

}

void Raven_Controller::updateRavenJointarray()
{
	//turn joint vector into a joint delta update
	//Cailbration linear function:	
	//jpos = (mv2jpos_rate[i]*(GreenSnake.mv(i-SHOULDER_GREEN) - calibrate_offset[i]))

	//Get jpos_desired initialiase as current jpos by default:
	for (int i = 0; i<14; i++){
		jpos_desired[i] = jpos_current[i];
	}

	//Extract motor values from snake raven class
	//Left arm
	for (int i = SHOULDER_GOLD; i < (SHOULDER_GOLD+GoldSnake.DOF); i++){
		jpos_desired[i] = (mv2jpos_rate[i]*(GoldSnake.mv(i-SHOULDER_GOLD) - calibrate_offset[i]));
	}
	//Right arm
	for (int i = SHOULDER_GREEN; i < (SHOULDER_GREEN+GreenSnake.DOF); i++){
		jpos_desired[i] = (mv2jpos_rate[i]*(GreenSnake.mv(i-SHOULDER_GREEN) - calibrate_offset[i]));
	}

	//Check the joints and move
	float error;
	for (int i = 0; i<14; i++){
		//check if the robot is there or not
		error = jpos_desired[i] - jpos_current[i];
		//send delta
		if ((i==Z_INS_GOLD)||(i==Z_INS_GREEN)){
			//prismatic
			if (fabs(error)<0.2){
				delta_joint[i] = 0; //filters the jittering
			}
			else{
				delta_joint[i] = saturate_round(error, motor_maxspeed_mm);
			}
		}
		else if ((i==SHOULDER_GOLD)||(i==ELBOW_GOLD)||(i==SHOULDER_GREEN)||(i==ELBOW_GREEN)){
			//slow revolute
			if (fabs(error)<deg2rad(0.5)){
				delta_joint[i] = 0; //filters the jittering
			}
			else{
				delta_joint[i] = saturate_round(error, motor_maxspeed_rad1);
			}
			//delta_joint[i] = saturate_round(error, motor_maxspeed_rad1);
		}
		else{
			//fast revolute
			if (fabs(error)<deg2rad(0.5)){
				delta_joint[i] = 0; //filters the jittering
			}
			else{
				delta_joint[i] = saturate_round(error, motor_maxspeed_rad2);
			}
		}

	}
}

double Raven_Controller::saturate_round(double input, float limit)
{
	double output;
	//round the input to a certain decimal place
	double value = round(input * 1000000); 

	output = value / 1000000.0;

	//saturate
	if (output>limit){
		output = limit;
	}
	else if (output<-limit){
		output = -limit;
	}

    return output;
}
